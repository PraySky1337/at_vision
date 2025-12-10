#include "detector_node.hpp"

// ROS
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.hpp>

// C++/OpenCV
#include <algorithm>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <string>

#include "inference/ov_armor_at.hpp"
#include "inference/ov_armor_tup.hpp"

namespace {
inline std::string resolve_pkg_url(const std::string& url) {
    const std::string pfx = "package://";
    if (url.rfind(pfx, 0) != 0)
        return url;
    auto rest = url.substr(pfx.size());
    auto pos  = rest.find('/');
    if (pos == std::string::npos)
        return url;
    auto pkg   = rest.substr(0, pos);
    auto rel   = rest.substr(pos + 1);
    auto share = ament_index_cpp::get_package_share_directory(pkg);
    return (std::filesystem::path(share) / rel).string();
}

cv::Scalar boxColor(int c) {
    switch (c) {
    case 2: return {160, 160, 160}; // 灰
    case 1: return {0, 0, 255};     // 红
    case 0: return {255, 0, 0};     // 蓝
    case 3: return {180, 0, 180};   // 紫
    default: return {0, 255, 0};    // UNKNOWN
    }
}

void fillRectAlpha(cv::Mat& img, const cv::Rect& r, const cv::Scalar& col, double alpha = 0.15) {
    if (r.width <= 0 || r.height <= 0)
        return;
    cv::Mat roi = img(r);
    cv::Mat color(roi.size(), roi.type(), col);
    cv::addWeighted(color, alpha, roi, 1.0 - alpha, 0.0, roi);
}

void drawCornerBox(cv::Mat& img, const cv::Rect& r, const cv::Scalar& col, int t) {
    int x = r.x, y = r.y, w = r.width, h = r.height;
    int L   = std::max(8, std::min(w, h) / 5);
    auto aa = cv::LINE_AA;
    cv::line(img, {x, y}, {x + L, y}, col, t, aa);
    cv::line(img, {x, y}, {x, y + L}, col, t, aa);
    cv::line(img, {x + w, y}, {x + w - L, y}, col, t, aa);
    cv::line(img, {x + w, y}, {x + w, y + L}, col, t, aa);
    cv::line(img, {x, y + h}, {x + L, y + h}, col, t, aa);
    cv::line(img, {x, y + h}, {x, y + h - L}, col, t, aa);
    // cv::line(img, {x + w, y + h}, {x + w - L, y + h}, col, t, aa);
    // cv::line(img, {x + w, y + h}, {x + w, y + h - L}, col, t, aa);
}

} // namespace

namespace rm_auto_aim {

ArmorDetectorOVNode::ArmorDetectorOVNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("armor_detector_ov", options) {

    RCLCPP_INFO(get_logger(), "ArmorDetectorOVNode initializing...");

    // —— 参数 ——
    model_name_   = this->declare_parameter<std::string>("model_name", "armor_tup");
    const auto mp = this->declare_parameter<std::string>(
        "model_path", "package://armor_detector_ov/model/armor_tup.xml");
    model_path_   = resolve_pkg_url(mp);
    debug_        = this->declare_parameter<bool>("debug", true);
    detect_color_ = this->declare_parameter<std::string>("detect_color", "RED"); // RED/BLUE
    odom_frame_   = this->declare_parameter<std::string>("target_frame", "odom");
    use_ba_       = this->declare_parameter<bool>("use_ba", true);
    const bool use_gpu =
        this->declare_parameter<bool>("use_gpu", false); // true: GPU, false: CPU (default)
    enable_multi_thread_ =
        this->declare_parameter<bool>("enable_multi_thread", false); // OpenVINO throughput mode
    device_name_ = use_gpu ? "GPU" : "CPU";

    model_ = OVModelManager::instance().create(model_name_);
    if (!model_) {
        RCLCPP_ERROR(get_logger(), "Create model '%s' failed.", model_name_.c_str());
    } else if (!model_->load(model_path_, device_name_, false, enable_multi_thread_)) {
        RCLCPP_ERROR(get_logger(), "Load OpenVINO model failed: %s", model_path_.c_str());
        model_.reset();
    } else {
        RCLCPP_INFO(
            get_logger(), "OpenVINO model loaded: %s (%s) on %s (multi_thread=%s)",
            model_path_.c_str(), model_name_.c_str(), device_name_.c_str(),
            enable_multi_thread_ ? "on" : "off");
    }

    // —— 发布者 ——
    armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
        "armor_detector/armors", rclcpp::SensorDataQoS());
    marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_detector/marker", 10);
    result_img_pub_ = image_transport::create_publisher(this, "armor_detector/result_img");

    // —— Marker 预设 ——
    armor_marker_.ns       = "armors";
    armor_marker_.action   = visualization_msgs::msg::Marker::ADD;
    armor_marker_.type     = visualization_msgs::msg::Marker::CUBE;
    armor_marker_.scale.x  = 0.03;
    armor_marker_.scale.y  = 0.15;
    armor_marker_.scale.z  = 0.125;
    armor_marker_.color.a  = 0.25;
    armor_marker_.color.r  = 1.0;
    armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    text_marker_.ns       = "classification";
    text_marker_.action   = visualization_msgs::msg::Marker::ADD;
    text_marker_.type     = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker_.scale.z  = 0.1;
    text_marker_.color.a  = 1.0;
    text_marker_.color.r  = 1.0;
    text_marker_.color.g  = 1.0;
    text_marker_.color.b  = 1.0;
    text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    // —— 相机内参（一次性） ——
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
            cam_center_ = cv::Point2f(
                static_cast<float>(camera_info->k[2]), static_cast<float>(camera_info->k[5]));
            cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
            cam_info_sub_.reset();
            pnp_solver_ = std::make_unique<PnPSolver>(cam_info_->k, cam_info_->d);
            RCLCPP_INFO(
                this->get_logger(), "Camera info received. fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                camera_info->k[0], camera_info->k[4], camera_info->k[2], camera_info->k[5]);
        });

    // —— 图像订阅 ——
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorOVNode::imageCallback, this, std::placeholders::_1));

    // —— 动态参数回调 ——
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ArmorDetectorOVNode::onSetParameters, this, std::placeholders::_1));

    // —— TF ——
    tf2_buffer_          = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    if (debug_) {
        RCLCPP_INFO(
            get_logger(), "Debug draw enabled: publish result image to /armor_detector/result_img");
    }
    RCLCPP_INFO(get_logger(), "ArmorDetectorOVNode initialized.");
}

ArmorDetectorOVNode::~ArmorDetectorOVNode() = default;

std::string ArmorDetectorOVNode::color_letter_(int color) {
    switch (color) {
    case 0: return "B";
    case 1: return "R";
    case 2: return "G";
    case 3: return "P";
    default: return "?";
    }
}

void ArmorDetectorOVNode::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    try {
        rclcpp::Time target_time = img_msg->header.stamp;
        auto odom_to_gimbal      = tf2_buffer_->lookupTransform(
            odom_frame_, img_msg->header.frame_id, target_time,
            rclcpp::Duration::from_seconds(0.01));
        auto msg_q = odom_to_gimbal.transform.rotation;
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3 tf2_matrix = tf2::Matrix3x3(tf_q);
        imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1], tf2_matrix.getRow(0)[2],
            tf2_matrix.getRow(1)[0], tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
            tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1], tf2_matrix.getRow(2)[2];
    } catch (...) {
        return;
    }

    if (!model_) {
        RCLCPP_ERROR(get_logger(), "Model not initialized");
        return;
    }

    cv::Mat img;
    try {
        img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
        return;
    }
    if (img.empty()) {
        RCLCPP_WARN(get_logger(), "Image empty");
        return;
    }

    std::vector<ArmorObject> dets;
    if (auto* armor_ov_tup = dynamic_cast<OVArmorTUP*>(model_.get())) {
        armor_ov_tup->detect(img, dets);
    } else if (auto* armor_ov_at = dynamic_cast<OVArmorAT*>(model_.get())) {
        armor_ov_at->detect(img, dets);
    } else
        (void)model_->run(img);

    if (debug_) {
        cv::Mat draw = img.clone();
        if (dets.size() > 20) {
            RCLCPP_WARN(get_logger(), "Too many armors detected: %d", (int)dets.size());
            dets.clear();
        }
        drawResults(draw, dets, cv::Rect());
        result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "bgr8", draw).toImageMsg());
    }
    // filteredArmors(dets);
    auto armors   = handleDets(dets);
    armors.header = img_msg->header;
    armors_pub_->publish(armors);
    publishMarkers(armors);
}

void ArmorDetectorOVNode::filteredArmors(std::vector<ArmorObject>& armors) {
    if (detect_color_ == "RED") {
        armors.erase(
            std::remove_if(
                armors.begin(), armors.end(), [](const ArmorObject& a) { return a.color != 1; }),
            armors.end());
    } else if (detect_color_ == "BLUE") {
        armors.erase(
            std::remove_if(
                armors.begin(), armors.end(), [](const ArmorObject& a) { return a.color != 0; }),
            armors.end());
    } else if (detect_color_ == "ALL") {
        // nothing todo
    } else {
        RCLCPP_WARN(get_logger(), "Unknown detect_color_: %s", detect_color_.c_str());
    }
}

rm_interfaces::msg::Armors ArmorDetectorOVNode::handleDets(std::vector<ArmorObject>& armors) {
    rm_interfaces::msg::Armors armors_msg;
    rm_interfaces::msg::Armor armor_msg;
    if (armors.size() > 20) {
        RCLCPP_WARN(get_logger(), "Too many armors detected: %d", (int)armors.size());
        return armors_msg;
    }
    if (!pnp_solver_) {
        return armors_msg;
    }
    cv::Mat rvec, tvec;
    for (const auto& a : armors) {
        if (use_ba_) {
            if (!pnp_solver_->solvePnPWithBA(a, imu_to_camera_, rvec, tvec)) {
                RCLCPP_ERROR(
                    get_logger(), "PnP failed for armor cls = %d, check armor model.", a.cls);
            }
        } else {
            if (!pnp_solver_->solvePnP(a, rvec, tvec)) {
                RCLCPP_ERROR(
                    get_logger(), "PnP failed for armor cls=%d, check armor model.", a.cls);
            }
        }
        cv::Mat Rcv; // 3x3 CV_64F
        cv::Rodrigues(rvec, Rcv);
        tf2::Matrix3x3 R(
            Rcv.at<double>(0, 0), Rcv.at<double>(0, 1), Rcv.at<double>(0, 2), Rcv.at<double>(1, 0),
            Rcv.at<double>(1, 1), Rcv.at<double>(1, 2), Rcv.at<double>(2, 0), Rcv.at<double>(2, 1),
            Rcv.at<double>(2, 2));
        tf2::Quaternion q;
        R.getRotation(q);
        q.normalize();
        armor_msg.pose.position.x    = tvec.at<double>(0);
        armor_msg.pose.position.y    = tvec.at<double>(1);
        armor_msg.pose.position.z    = tvec.at<double>(2);
        armor_msg.pose.orientation.x = q.x();
        armor_msg.pose.orientation.y = q.y();
        armor_msg.pose.orientation.z = q.z();
        armor_msg.pose.orientation.w = q.w();
        armor_msg.number             = armor_cls_to_string(a.cls);
        armor_msg.type =
            armor_msg.number == "1" || armor_msg.number == "base_large" ? "large" : "small";
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(
            cv::Point2f(static_cast<float>(cam_info_->k[2]), static_cast<float>(cam_info_->k[5])));
        armors_msg.armors.emplace_back(armor_msg);
    }
    return armors_msg;
}

// ============ 动态参数 ============
rcl_interfaces::msg::SetParametersResult
    ArmorDetectorOVNode::onSetParameters(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    for (const auto& p : parameters) {
        if (p.get_name() == "debug") {
            debug_ = p.as_bool();
            RCLCPP_INFO(get_logger(), "debug = %s", debug_ ? "true" : "false");
        } else if (p.get_name() == "detect_color") {
            auto v = p.as_string();
            std::transform(v.begin(), v.end(), v.begin(), ::toupper);
            if (v == "RED" || v == "BLUE") {
                detect_color_ = v;
                RCLCPP_INFO(get_logger(), "detect_color = %s", detect_color_.c_str());
            } else {
                res.successful = false;
                res.reason     = "detect_color must be RED/BLUE";
            }
        } else if (p.get_name() == "model_path") {
            RCLCPP_WARN(get_logger(), "Changing model_path at runtime is not supported.");
        } else if (p.get_name() == "model_name") {
            RCLCPP_WARN(get_logger(), "Changing model_name at runtime is not supported.");
        } else if (p.get_name() == "use_gpu" || p.get_name() == "enable_multi_thread") {
            res.successful = false;
            res.reason =
                "Changing inference device/threading at runtime is not supported. Restart node.";
        }
    }
    return res;
}

void ArmorDetectorOVNode::publishMarkers(const rm_interfaces::msg::Armors& armors_msg) noexcept {
    using visualization_msgs::msg::Marker;

    // 每次先清空，否则 markers 会越堆越多
    marker_array_.markers.clear();

    // 没有装甲板时，发一个 DELETEALL 清空 RViz 里的旧标记
    if (armors_msg.armors.empty()) {
        Marker clear_marker;
        clear_marker.header = armors_msg.header;
        clear_marker.ns     = "armors";
        clear_marker.id     = 0;
        clear_marker.action = Marker::DELETEALL;
        marker_array_.markers.emplace_back(std::move(clear_marker));

        marker_pub_->publish(marker_array_);
        return;
    }

    // 有装甲板时，基于模板 marker（armor_marker_）生成
    int id = 0;
    for (const auto& a : armors_msg.armors) {
        Marker marker = armor_marker_;     // armor_marker_ 里预先配置好 type/scale/color 等
        marker.header = armors_msg.header; // 保证时间戳和坐标系正确
        marker.id     = id++;              // RViz 要求 (ns,id) 唯一
        marker.pose   = a.pose;

        // 如果有想显示的额外信息，可以在这里设置 text / color 等
        // marker.text = std::to_string(a.id);  // 示例

        marker_array_.markers.emplace_back(std::move(marker));
    }

    marker_pub_->publish(marker_array_);
}
// ============ 可视化 ============
void ArmorDetectorOVNode::drawResults(
    cv::Mat& src, const std::vector<ArmorObject>& armor_objects, const cv::Rect& roi) noexcept {
    const int img_min = std::max(1, std::min(src.cols, src.rows));
    for (const auto& obj : armor_objects) {
        const cv::Rect img_rc(0, 0, src.cols, src.rows);
        cv::Rect r(
            cvRound(obj.rect.x), cvRound(obj.rect.y), cvRound(obj.rect.width),
            cvRound(obj.rect.height));
        r &= img_rc;
        if (r.empty())
            continue;

        const cv::Scalar color = boxColor(obj.color);
        const int thick        = std::clamp(img_min / 600, 2, 6);
        const double fscale    = std::clamp(img_min / 900.0, 0.8, 1.5);
        const int fthick       = std::clamp(thick - 1, 1, 3);

        fillRectAlpha(src, r, color, 0.15);
        // drawCornerBox(src, r, color, thick);

        // 四点框
        for (int i = 0; i < 4; ++i)
            cv::line(src, obj.apex[i], obj.apex[(i + 1) % 4], color, thick, cv::LINE_AA);

        // 标签：Color + id + conf
        std::string label =
            cv::format("%s %d %.2f%%", color_letter_(obj.color).c_str(), obj.cls, obj.prob * 100.0);

        int base    = 0;
        cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, fscale, fthick, &base);
        int pad     = 6;
        cv::Point tl(r.x, std::max(0, r.y - ts.height - base - 2 * pad));
        cv::Rect lb(tl.x, tl.y, ts.width + 2 * pad, ts.height + base + 2 * pad);
        lb &= cv::Rect(0, 0, src.cols, src.rows);
        if (lb.width > 0 && lb.height > 0) {
            cv::Mat roi = src(lb);
            cv::Mat bg(roi.size(), roi.type(), color);
            cv::addWeighted(bg, 0.8, roi, 0.2, 0.0, roi);
            cv::putText(
                src, label, {lb.x + pad, lb.y + pad + ts.height}, cv::FONT_HERSHEY_SIMPLEX, fscale,
                {0, 0, 0}, fthick + 2, cv::LINE_AA);
            cv::putText(
                src, label, {lb.x + pad, lb.y + pad + ts.height}, cv::FONT_HERSHEY_SIMPLEX, fscale,
                {255, 255, 255}, fthick, cv::LINE_AA);
        }
    }
    cv::rectangle(src, roi, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

} // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorOVNode)
