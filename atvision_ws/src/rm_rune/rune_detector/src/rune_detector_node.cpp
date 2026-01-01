#include "rune_detector/rune_detector_node.hpp"
// ros2
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <opencv2/highgui.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <rmw/qos_profiles.h>

#include <rclcpp/qos.hpp>
// std
#include <algorithm>
#include <array>
#include <filesystem>
#include <numeric>
#include <vector>
// third party
#include "rune_detector/types.hpp"
#include <opencv2/imgproc.hpp>

namespace fyt::rune {

RuneDetectorNode::RuneDetectorNode(const rclcpp::NodeOptions& options)
    : Node("rune_detector", options)
    , is_rune_(false)
    , tf_buffer_(this->get_node_clock_interface()->get_clock())
    , tf_listener_(tf_buffer_) {
    std::cerr << "Starting RuneDetectorNode!" << std::endl;

    frame_id_ = declare_parameter("frame_id", "camera_optical_frame");

    int detect_color_param = this->declare_parameter<int>("detect_color", 0);

    if (detect_color_param == 0) {
        detect_color_ = EnemyColor::RED;
    } else if (detect_color_param == 1) {
        detect_color_ = EnemyColor::BLUE;
    } else {
        detect_color_ = EnemyColor::WHITE;
        std::cerr << "Invalid detect_color param, set to UNKNOWN" << std::endl;
    }
    // Detector
    rune_detector_ = initDetector();

    // Debug参数
    debug_          = declare_parameter("debug", true);
    publish_binary_ = declare_parameter("publish_binary", true);

    if (debug_) {
        createDebugPublishers();
    }

    // 图像订阅
    auto qos      = rclcpp::SensorDataQoS();
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);

            cameraMatrix_ =
                cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_info->k.data())).clone();
            distCoeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double*>(camera_info->d.data())).clone();
            cam_info_sub_.reset();
        });

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", qos,
        std::bind(&RuneDetectorNode::imageCallback, this, std::placeholders::_1));

    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "rune_detector/marker", rclcpp::SensorDataQoS());
}

RuneDetectorNode::~RuneDetectorNode() { std::cerr << "Stopping video thread..." << std::endl; }

std::unique_ptr<RuneDetector> RuneDetectorNode::initDetector() {

    arrow_threshold_   = this->declare_parameter("detector.arrow_threshold", 170);
    target_threshold_  = this->declare_parameter("detector.target_threshold", 130);
    rcenter_threshold_ = this->declare_parameter("detector.rcenter_threshold", 120);
    // Set dynamic parameter callback
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        std::vector<rclcpp::Parameter> parameters);
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&RuneDetectorNode::onSetParameters, this, std::placeholders::_1));

    // Create detector
    auto rune_detector = std::make_unique<RuneDetector>(
        detect_color_, arrow_threshold_, target_threshold_, rcenter_threshold_);

    return rune_detector;
}

void RuneDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {

    // 转换 ROS 图像为 OpenCV
    cv::Mat src_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    if (src_img.empty()) {
        std::cerr << "[ERROR] imageCallback: Received empty image" << std::endl;
        ;
        return;
    }

    int image_width  = msg->width;
    int image_height = msg->height;
    auto timestamp   = rclcpp::Time(msg->header.stamp);
    frame_id_        = msg->header.frame_id;

    // 检测
    rune_detector_->detect(src_img, image_width, image_height);

    if (!rune_detector_->targets.empty() && rune_detector_->rcenter.center != cv::Point2f(0, 0)) {
        rune_detector_->setKeyPoints();
    }

    Eigen::Matrix4d pose;
    if (rune_detector_->rcenter.center != cv::Point2f(0, 0) && !rune_detector_->targets.empty()
        && !cameraMatrix_.empty() && !distCoeffs_.empty()) {

        pose = rune_detector_->solve(cameraMatrix_, distCoeffs_, timestamp, tf_buffer_);
    }

    // Debug 可视化
    if (debug_) {
        cv::Mat debug_img = src_img.clone();

        if (!rune_detector_->arrows.empty()) {
            for (auto arrow : rune_detector_->arrows) {
                cv::Point2f ves[4];
                arrow.rotated.points(ves);
                for (size_t i = 0; i < 4; i++) {
                    cv::line(
                        debug_img, ves[i] + rune_detector_->globalRoi.tl(),
                        ves[(i + 1) % 4] + rune_detector_->globalRoi.tl(),
                        cv::Scalar(255, 255, 255), 2);
                }
                cv::circle(debug_img, arrow.center, 3, cv::Scalar(255, 0, 255), -1);
            }
        }

        if (!rune_detector_->targets.empty()) {
            for (auto target : rune_detector_->targets) {
                if (target.center != cv::Point2f(0, 0)) {
                    cv::circle(
                        debug_img, target.center, 3, cv::Scalar(0, 255, 255), -1); // 黄 - 中心
                }

                if (target.keypnt.ru != cv::Point2f(0, 0) || target.keypnt.rd != cv::Point2f(0, 0)
                    || target.keypnt.ld != cv::Point2f(0, 0)
                    || target.keypnt.lu != cv::Point2f(0, 0)) {
                    // 1. 绘制四个角点
                    cv::circle(
                        debug_img, target.keypnt.ru, 3, cv::Scalar(255, 0, 0), -1); // 蓝 - 右上
                    cv::circle(
                        debug_img, target.keypnt.rd, 3, cv::Scalar(0, 255, 0), -1); // 绿 - 右下
                    cv::circle(
                        debug_img, target.keypnt.ld, 3, cv::Scalar(0, 0, 255), -1); // 红 - 左下
                    cv::circle(
                        debug_img, target.keypnt.lu, 3, cv::Scalar(0, 0, 0), -1);   // 黑 - 左上

                    // 2. 绘制矩形边框（按「右上→右下→左下→左上→右上」闭环顺序）
                    cv::line(
                        debug_img, target.keypnt.ru, target.keypnt.rd, cv::Scalar(255, 255, 255),
                        2); // 右上→右下
                    cv::line(
                        debug_img, target.keypnt.rd, target.keypnt.ld, cv::Scalar(255, 255, 255),
                        2); // 右下→左下
                    cv::line(
                        debug_img, target.keypnt.ld, target.keypnt.lu, cv::Scalar(255, 255, 255),
                        2); // 左下→左上
                    cv::line(
                        debug_img, target.keypnt.lu, target.keypnt.ru, cv::Scalar(255, 255, 255),
                        2); // 左上→右上
                }
            }
        }

        if (rune_detector_->rcenter.center != cv::Point2f(0, 0)) {
            cv::circle(debug_img, rune_detector_->rcenter.center, 3, cv::Scalar(255, 255, 0), -1);
        }

        auto&& result = cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg();
        result_img_pub_.publish(std::move(result));

        // Publish binary images (coordinates are relative to globalRoi)
        if (publish_binary_) {
            cv::Mat arrow_binary_with_roi = rune_detector_->arrowImg.clone();
            if (arrow_binary_with_roi.channels() == 1) {
                cv::cvtColor(arrow_binary_with_roi, arrow_binary_with_roi, cv::COLOR_GRAY2BGR);
            }
            // Draw centerRoi first (red), then targetROIs (green) on top
            if (rune_detector_->centerRoi.width > 0 && rune_detector_->centerRoi.height > 0) {
                cv::rectangle(
                    arrow_binary_with_roi, rune_detector_->centerRoi, cv::Scalar(0, 0, 255), 2);
            }
            // Draw targetROIs on top (green with thicker line)
            for (size_t i = 0; i < rune_detector_->targetROIs.size(); i++) {
                cv::rectangle(
                    arrow_binary_with_roi, rune_detector_->targetROIs[i], cv::Scalar(0, 255, 0), 3);
            }
            auto&& arrow_binary_result =
                cv_bridge::CvImage(msg->header, "bgr8", arrow_binary_with_roi).toImageMsg();
            arrow_binary_pub_.publish(std::move(arrow_binary_result));

            cv::Mat target_binary_with_roi = rune_detector_->targetImg.clone();
            if (target_binary_with_roi.channels() == 1) {
                cv::cvtColor(target_binary_with_roi, target_binary_with_roi, cv::COLOR_GRAY2BGR);
            }
            // Draw centerRoi first (red), then targetROIs (green) on top
            if (rune_detector_->centerRoi.width > 0 && rune_detector_->centerRoi.height > 0) {
                cv::rectangle(
                    target_binary_with_roi, rune_detector_->centerRoi, cv::Scalar(0, 0, 255), 2);
            }
            // Draw targetROIs on top (green with thicker line)
            for (size_t i = 0; i < rune_detector_->targetROIs.size(); i++) {
                cv::rectangle(
                    target_binary_with_roi, rune_detector_->targetROIs[i], cv::Scalar(0, 255, 0),
                    3);
            }
            auto&& target_binary_result =
                cv_bridge::CvImage(msg->header, "bgr8", target_binary_with_roi).toImageMsg();
            target_binary_pub_.publish(std::move(target_binary_result));

            cv::Mat rcenter_binary_with_roi = rune_detector_->rCenterImg.clone();
            if (rcenter_binary_with_roi.channels() == 1) {
                cv::cvtColor(rcenter_binary_with_roi, rcenter_binary_with_roi, cv::COLOR_GRAY2BGR);
            }
            // Draw centerRoi first (red), then targetROIs (green) on top
            if (rune_detector_->centerRoi.width > 0 && rune_detector_->centerRoi.height > 0) {
                cv::rectangle(
                    rcenter_binary_with_roi, rune_detector_->centerRoi, cv::Scalar(0, 0, 255), 2);
            }
            // Draw targetROIs on top (green with thicker line)
            for (size_t i = 0; i < rune_detector_->targetROIs.size(); i++) {
                cv::rectangle(
                    rcenter_binary_with_roi, rune_detector_->targetROIs[i], cv::Scalar(0, 255, 0),
                    3);
            }
            auto&& rcenter_binary_result =
                cv_bridge::CvImage(msg->header, "bgr8", rcenter_binary_with_roi).toImageMsg();
            rcenter_binary_pub_.publish(std::move(rcenter_binary_result));
        }
    }
    visualization_msgs::msg::MarkerArray marker_array;

    // 检查R标和靶体数据是否有效
    if (pose.isZero() || pose == Eigen::Matrix4d::Identity()) {
        std::cerr << "[ERROR] imageCallback: pose is invalid (zero or identity)" << std::endl;
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);
        marker_array_pub_->publish(marker_array);
        return;
    }

    Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
    Eigen::Quaterniond q(R);

    if (rune_detector_->targets.size() == 1) {

        Eigen::Vector3d target_rel_rune(0.0f, 0.0f, RUNE_R2PANCENTER);
        Eigen::Vector3d r_center_world(pose(0, 3), pose(1, 3), pose(2, 3));
        Eigen::Vector3d target_center_world = r_center_world + R * target_rel_rune;

        visualization_msgs::msg::Marker target_body = create_target_marker(
            timestamp, target_center_world, q, 0.3, 0.3, 0.1, 1.0, 0.0, 0.0, 0.8, 0);
        marker_array.markers.push_back(target_body);

        visualization_msgs::msg::Marker r_marker = create_r_marker(timestamp, r_center_world);
        marker_array.markers.push_back(r_marker);
    } else if (rune_detector_->targets.size() == 2) {

        // 第一个目标（同单目标逻辑，ID=0）
        Eigen::Vector3d r_center_world(pose(0, 3), pose(1, 3), pose(2, 3));
        Eigen::Vector3d target_rel_rune0(0.0f, 0.0f, RUNE_R2PANCENTER);
        Eigen::Vector3d target_center0_world = r_center_world + R * target_rel_rune0;

        visualization_msgs::msg::Marker target_body0 = create_target_marker(
            timestamp, target_center0_world, q, 0.3, 0.3, 0.1, 1.0, 0.0, 0.0, 0.8, 0);
        marker_array.markers.push_back(target_body0);

        // 第二个目标（ID=1）
        // 计算第二个目标的R中心世界坐标（基于roll旋转后的相对位置）
        double roll = rune_detector_->targets[0].roll; // id来自addOther逻辑，需保证可访问
        Eigen::Vector3d target_rel_rune1(0.0f, 0.0f, RUNE_R2PANCENTER);
        // 应用roll旋转（X轴）
        Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
        Eigen::Vector3d target_center1_world = r_center_world + R * (roll_angle * target_rel_rune1);

        visualization_msgs::msg::Marker target_body1 = create_target_marker(
            timestamp, target_center1_world, q, 0.3, 0.3, 0.1, 0.0, 1.0, 0.0, 0.8,
            1); // 绿色区分，ID=1
        marker_array.markers.push_back(target_body1);

        visualization_msgs::msg::Marker r_marker = create_r_marker(timestamp, r_center_world);
        marker_array.markers.push_back(r_marker);
    }
    marker_array_pub_->publish(marker_array);
}

rcl_interfaces::msg::SetParametersResult
    RuneDetectorNode::onSetParameters(std::vector<rclcpp::Parameter> parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    for (const auto& param : parameters) {
        if (param.get_name() == "arrow_threshold") {
            arrow_threshold_ = param.as_int();
        } else if (param.get_name() == "target_threshold") {
            target_threshold_ = param.as_int();
        } else if (param.get_name() == "rcenter_threshold") {
            rcenter_threshold_ = param.as_int();
        } else if (param.get_name() == "debug") {
            debug_ = param.as_bool();
        } else if (param.get_name() == "publish_binary") {
            publish_binary_ = param.as_bool();
        }
    }
    result.successful = true;
    return result;
}

void RuneDetectorNode::createDebugPublishers() {
    result_img_pub_ = image_transport::create_publisher(this, "rune_detector/result_img");
    if (publish_binary_) {
        arrow_binary_pub_  = image_transport::create_publisher(this, "rune_detector/arrow_binary");
        target_binary_pub_ = image_transport::create_publisher(this, "rune_detector/target_binary");
        rcenter_binary_pub_ =
            image_transport::create_publisher(this, "rune_detector/rcenter_binary");
    }
}

void RuneDetectorNode::destroyDebugPublishers() {
    result_img_pub_.shutdown();
    if (arrow_binary_pub_) {
        arrow_binary_pub_.shutdown();
    }
    if (target_binary_pub_) {
        target_binary_pub_.shutdown();
    }
    if (rcenter_binary_pub_) {
        rcenter_binary_pub_.shutdown();
    }
}

} // namespace fyt::rune
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::rune::RuneDetectorNode)
