// detector_node.cpp
#include "armor_detector/detector_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <auto_aim_interfaces/msg/detail/debug_armors__struct.hpp>
#include <auto_aim_interfaces/msg/detail/debug_lights__struct.hpp>
#include <hikcamera/image_capturer.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "armor_detector/pnp_solver.hpp"

namespace rm_auto_aim {

ArmorDetectorNode::ArmorDetectorNode(
    const std::string& name, const std::string& ns, const rclcpp::NodeOptions& options)
    : Node("armor_detector", options)
    , hik_camera_params_(this)
    , cam_info_manager_(this) {
    RCLCPP_INFO(get_logger(), "Starting ArmorDetectorNode...");
    img_capturer_ = std::make_unique<hikcamera::ImageCapturer>(
        hik_camera_params_.camera_profile, hik_camera_params_.camera_name.c_str());
    RCLCPP_INFO(get_logger(), "Camera initialized.");

    initDetectors();
    RCLCPP_INFO(get_logger(), "Detector initialized.");

    armors_pub_ =
        create_publisher<auto_aim_interfaces::msg::Armors>("~/armors", rclcpp::SensorDataQoS());
    marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", rclcpp::QoS(10));
    RCLCPP_INFO(get_logger(), "Publishers created.");

    debug_enabled_ = declare_parameter("debug", false);

    cam_info_manager_.setCameraName(hik_camera_params_.camera_name);
    cam_info_manager_.loadCameraInfo(hik_camera_params_.cam_info_url);
    auto cam_info = cam_info_manager_.getCameraInfo();
    pnp_solver_   = std::make_shared<PnPSolver>(cam_info.k, cam_info.d);
    cam_center_   = cv::Point2f(cam_info.k[2], cam_info.k[5]);

    param_cb_handle_ = add_on_set_parameters_callback(
        std::bind(&ArmorDetectorNode::onParametersSet, this, std::placeholders::_1));

    if (debug_enabled_) {
        debug_pubs_ = std::make_shared<DebugPublishers>(this);
    }

    detect_thread = std::make_unique<std::thread>([this]() {
        RCLCPP_INFO(get_logger(), "detect loop thread running");
        detectLoop();
    });
}

ArmorDetectorNode::~ArmorDetectorNode() {
    if (detect_thread && detect_thread->joinable()) {
        detect_thread->join();
        detect_thread.reset();
    }
    rclcpp::shutdown();
}

void ArmorDetectorNode::detectLoop() {
    cv::Mat img;
    std_msgs::msg::Header header;
    header.frame_id = "camera_optical_frame";
    while (rclcpp::ok()) {
        img          = img_capturer_->read();
        header.stamp = now();
        detectOnce(img, header);
        asm volatile("");
    }
}

void ArmorDetectorNode::detectOnce(const cv::Mat& raw_img, const std_msgs::msg::Header& header) {
    auto armors = detector_->detect(raw_img);
    if (debug_enabled_) {
        cv::Mat dbg_img = raw_img.clone();
        debug_pubs_->binary.publish(
            cv_bridge::CvImage(header, "mono8", detector_->binary_img).toImageMsg());
        debug_pubs_->lights->publish(detector_->debug_lights);
        debug_pubs_->armors->publish(detector_->debug_armors);
        debug_pubs_->numbers.publish(
            *cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", detector_->getAllNumbersImage())
                 .toImageMsg());
        detector_->drawResults(dbg_img);
        cv::circle(dbg_img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
        debug_pubs_->result.publish(cv_bridge::CvImage(header, "bgr8", dbg_img).toImageMsg());
    }
    if (armors.empty())
        return;
    publishArmorsAndMarkers(armors, header);
}

void ArmorDetectorNode::initDetectors() {
    rcl_interfaces::msg::ParameterDescriptor pd;
    pd.integer_range.resize(1);
    pd.integer_range[0].step       = 1;
    pd.integer_range[0].from_value = 0;
    pd.integer_range[0].to_value   = 255;

    int binary_thres = declare_parameter("binary_thres", 160, pd);

    pd.description               = "0-RED, 1-BLUE";
    pd.integer_range[0].to_value = 1;
    auto detect_color            = declare_parameter("detect_color", RED, pd);

    Detector::LightParams l_params{
        .min_ratio = declare_parameter("light.min_ratio", 0.1),
        .max_ratio = declare_parameter("light.max_ratio", 0.4),
        .max_angle = declare_parameter("light.max_angle", 40.0)};

    Detector::ArmorParams a_params{
        .min_light_ratio           = declare_parameter("armor.min_light_ratio", 0.7),
        .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
        .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
        .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
        .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
        .max_angle                 = declare_parameter("armor.max_angle", 35.0)};

    std::string pkg_path   = ament_index_cpp::get_package_share_directory("armor_detector");
    std::string model_path = pkg_path + "/model/mlp.onnx";
    std::string label_path = pkg_path + "/model/label.txt";
    double threshold       = declare_parameter("classifier_threshold", 0.7);
    auto ignore_classes = declare_parameter("ignore_classes", std::vector<std::string>{"negative"});

    detector_ = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);
    detector_->classifier =
        std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);
}

void ArmorDetectorNode::publishArmorsAndMarkers(const std::vector<Armor>& armors, const std_msgs::msg::Header& header) {
    auto_aim_interfaces::msg::Armors msg;
    msg.header = header;

    int id = 0;
    visualization_msgs::msg::MarkerArray marray;
    for (const auto& armor : armors) {
        cv::Mat rvec, tvec;
        bool ok = pnp_solver_ && pnp_solver_->solvePnP(armor, rvec, tvec);
        if (!ok) {
            RCLCPP_WARN(get_logger(), "PnP failed for armor");
            continue;
        }
        auto amsg   = auto_aim_interfaces::msg::Armor();
        amsg.type   = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        amsg.number = armor.number;
        geometry_msgs::msg::Pose pose;
        pose.position.x = tvec.at<double>(0);
        pose.position.y = tvec.at<double>(1);
        pose.position.z = tvec.at<double>(2);
        cv::Mat rot;
        cv::Rodrigues(rvec, rot);
        tf2::Matrix3x3 m(
            rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2), rot.at<double>(1, 0),
            rot.at<double>(1, 1), rot.at<double>(1, 2), rot.at<double>(2, 0), rot.at<double>(2, 1),
            rot.at<double>(2, 2));
        tf2::Quaternion q;
        m.getRotation(q);
        pose.orientation              = tf2::toMsg(q);
        amsg.pose                     = pose;
        amsg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);
        msg.armors.push_back(amsg);

        if (debug_enabled_) {
            visualization_msgs::msg::Marker cube, text;
            cube.header   = msg.header;
            cube.ns       = "armors";
            cube.type     = visualization_msgs::msg::Marker::CUBE;
            cube.action   = visualization_msgs::msg::Marker::ADD;
            cube.scale.x  = 0.05;
            cube.scale.z  = 0.125;
            cube.color.a  = 1.0f;
            cube.color.g  = 0.5f;
            cube.color.b  = 1.0f;
            cube.lifetime = rclcpp::Duration::from_seconds(0.1);

            text.header   = header;
            text.ns       = "classification";
            text.type     = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action   = visualization_msgs::msg::Marker::ADD;
            text.scale.z  = 0.1;
            text.color.a  = 1.0f;
            text.color.r  = 1.0f;
            text.color.g  = 1.0f;
            text.color.b  = 1.0f;
            text.lifetime = cube.lifetime;
            cube.id       = id++;
            cube.pose     = pose;
            marray.markers.push_back(cube);

            text.id   = id++;
            text.text = armor.classfication_result;
            text.pose = pose;
            text.pose.position.y -= 0.1;
            marray.markers.push_back(text);
        }
    }

    if (debug_enabled_) {
        marker_pub_->publish(marray);
    }
    armors_pub_->publish(msg);
}

rcl_interfaces::msg::SetParametersResult
    ArmorDetectorNode::onParametersSet(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason     = "";
    hikcamera::ImageCapturer::CameraProfile new_profile;
    bool changed = false;

    for (const auto& p : params) {
        const auto& name = p.get_name();
        if (name == "debug") {
            bool en        = p.as_bool();
            debug_enabled_ = en;
            if (en) {
                debug_pubs_ = std::make_shared<DebugPublishers>(this);
            } else {
                debug_pubs_.reset();
            }
        } else if (name == "binary_thres") {
            detector_->binary_thres = p.as_int();
        } else if (name == "detect_color") {
            detector_->detect_color = p.as_int();
        } else if (name == "classifier_threshold") {
            detector_->classifier->threshold = p.as_double();
        } else if (name == "light.min_ratio") {
            detector_->l.min_ratio = p.as_double();
        } else if (name == "light.max_ratio") {
            detector_->l.max_ratio = p.as_double();
        } else if (name == "light.max_angle") {
            detector_->l.max_angle = p.as_double();
        } else if (name == "armor.min_light_ratio") {
            detector_->a.min_light_ratio = p.as_double();
        } else if (name == "armor.min_small_center_distance") {
            detector_->a.min_small_center_distance = p.as_double();
        } else if (name == "armor.max_small_center_distance") {
            detector_->a.max_small_center_distance = p.as_double();
        } else if (name == "armor.min_large_center_distance") {
            detector_->a.min_large_center_distance = p.as_double();
        } else if (name == "armor.max_large_center_distance") {
            detector_->a.max_large_center_distance = p.as_double();
        } else if (name == "armor.max_angle") {
            detector_->a.max_angle = p.as_double();
        } else if (name == "camera.exposure_time") {
            new_profile.exposure_time =
                std::chrono::microseconds(static_cast<int>(p.as_double() * 1000.0));
            changed = true;
        } else if (name == "camera.gain") {
            new_profile.gain = static_cast<float>(p.as_double());
            changed          = true;
        } else if (name == "camera.invert") {
            new_profile.invert_image = p.as_bool();
            changed                  = true;
        } else if (name == "camera.trigger_mode") {
            new_profile.trigger_mode = p.as_bool();
            changed                  = true;
        }
        if (changed) {
            RCLCPP_INFO(get_logger(), "Updated camera profile dynamically.");
        }
    }
    return result;
}

DebugPublishers::DebugPublishers(rclcpp::Node* node_ptr) {
    this->lights =
        node_ptr->create_publisher<auto_aim_interfaces::msg::DebugLights>("~/debug_lights", 10);
    this->armors =
        node_ptr->create_publisher<auto_aim_interfaces::msg::DebugArmors>("~/debug_armors", 10);
    this->binary  = image_transport::create_publisher(node_ptr, "~/binary_img");
    this->img_raw = image_transport::create_publisher(node_ptr, "~/image_raw");
    this->numbers = image_transport::create_publisher(node_ptr, "~/number_img");
    this->result  = image_transport::create_publisher(node_ptr, "~/result_img");
}

DebugPublishers::~DebugPublishers() {
    binary.shutdown();
    img_raw.shutdown();
    numbers.shutdown();
    result.shutdown();
}

} // namespace rm_auto_aim
