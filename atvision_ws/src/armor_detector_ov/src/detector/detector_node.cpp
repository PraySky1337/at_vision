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
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <opencv2/opencv.hpp>
#include <optional>
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

bool isLargeTarget(const rm_interfaces::msg::Target& target) {
    return target.id == "1" || target.id == "base_large";
}

cv::Rect2f unionRect(const cv::Rect2f& a, const cv::Rect2f& b) {
    const float x1 = std::min(a.x, b.x);
    const float y1 = std::min(a.y, b.y);
    const float x2 = std::max(a.x + a.width, b.x + b.width);
    const float y2 = std::max(a.y + a.height, b.y + b.height);
    return {x1, y1, x2 - x1, y2 - y1};
}

cv::Rect makeSquareRoi(const cv::Rect2f& rect, int min_side, const cv::Size& image_size) {
    if (image_size.width <= 0 || image_size.height <= 0) {
        return {};
    }

    const float cx = rect.x + rect.width * 0.5f;
    const float cy = rect.y + rect.height * 0.5f;
    float side     = std::max(rect.width, rect.height);
    side           = std::max(side, static_cast<float>(min_side));

    const float max_side = static_cast<float>(std::min(image_size.width, image_size.height));
    side                 = std::min(side, max_side);

    const int iside = std::max(1, static_cast<int>(std::ceil(side)));
    int x           = static_cast<int>(std::round(cx - iside * 0.5f));
    int y           = static_cast<int>(std::round(cy - iside * 0.5f));

    x = std::clamp(x, 0, image_size.width - iside);
    y = std::clamp(y, 0, image_size.height - iside);
    return {x, y, iside, iside};
}

int modelInputSize(const rm_auto_aim::OVModelBase* model) {
    if (dynamic_cast<const rm_auto_aim::OVArmorTUP*>(model)) {
        return rm_auto_aim::OVArmorTUP::INPUT_W;
    }
    if (dynamic_cast<const rm_auto_aim::OVArmorAT*>(model)) {
        return rm_auto_aim::OVArmorAT::INPUT_W;
    }
    return 416;
}

cv::Mat cameraMatrixFromInfo(const sensor_msgs::msg::CameraInfo& info) {
    cv::Mat K          = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = info.k[0];
    K.at<double>(0, 2) = info.k[2];
    K.at<double>(1, 1) = info.k[4];
    K.at<double>(1, 2) = info.k[5];
    return K;
}

cv::Mat distCoeffsFromInfo(const sensor_msgs::msg::CameraInfo& info) {
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
    for (size_t i = 0; i < std::min<size_t>(5, info.d.size()); ++i) {
        D.at<double>(0, static_cast<int>(i)) = info.d[i];
    }
    return D;
}

std::vector<std::array<tf2::Vector3, 4>>
    buildArmorCornersTargetFrame(const rm_interfaces::msg::Target& target) {
    // Armor plate size (mm -> m), aligned with pnp_solver.cpp defaults
    constexpr double SMALL_W = 135.0 / 1000.0;
    constexpr double SMALL_H = 55.0 / 1000.0;
    constexpr double LARGE_W = 230.0 / 1000.0;
    constexpr double LARGE_H = 55.0 / 1000.0;

    const bool large    = isLargeTarget(target);
    const double half_w = (large ? LARGE_W : SMALL_W) * 0.5;
    const double half_h = (large ? LARGE_H : SMALL_H) * 0.5;

    // corners in armor frame: x=0 plane, y left, z up
    const std::array<tf2::Vector3, 4> corners_armor = {
        tf2::Vector3(0, +half_w, +half_h),
        tf2::Vector3(0, +half_w, -half_h),
        tf2::Vector3(0, -half_w, -half_h),
        tf2::Vector3(0, -half_w, +half_h),
    };

    const int n = target.armors_num;
    if (n != 3 && n != 4) {
        return {};
    }

    std::vector<std::array<tf2::Vector3, 4>> corners;
    corners.reserve(static_cast<size_t>(n));

    const double angle_step = 2.0 * M_PI / static_cast<double>(n);
    const double cx         = target.position.x;
    const double cy         = target.position.y;
    const double z0         = target.position.z;

    for (int i = 0; i < n; ++i) {
        const bool another_pair = (n == 4) && (i == 1 || i == 3);
        const double r          = another_pair ? target.radius1 : target.radius0;
        const double angle      = target.yaw + angle_step * static_cast<double>(i);

        const double armor_x = cx - r * std::cos(angle);
        const double armor_y = cy - r * std::sin(angle);

        double armor_z = z0;
        if (n == 3) {
            armor_z = (i == 0) ? z0 : (i == 1) ? target.z1 : target.z2;
        } else { // n == 4
            armor_z = another_pair ? target.z1 : z0;
        }

        tf2::Transform T;
        T.setOrigin(tf2::Vector3(armor_x, armor_y, armor_z));
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, angle);
        T.setRotation(q);

        std::array<tf2::Vector3, 4> c{};
        for (size_t k = 0; k < 4; ++k) {
            c[k] = T * corners_armor[k];
        }
        corners.emplace_back(std::move(c));
    }

    return corners;
}

std::optional<cv::Rect2f> projectArmorToImageRect(
    const std::array<tf2::Vector3, 4>& corners_target, const tf2::Transform& target_to_cam,
    const cv::Mat& K, const cv::Mat& D) {
    std::vector<cv::Point3f> pts_cam;
    pts_cam.reserve(4);
    for (const auto& p_tgt : corners_target) {
        const tf2::Vector3 p_cam = target_to_cam * p_tgt;
        if (!std::isfinite(p_cam.x()) || !std::isfinite(p_cam.y()) || !std::isfinite(p_cam.z())) {
            return std::nullopt;
        }
        if (p_cam.z() <= 1e-6) {
            return std::nullopt;
        }
        pts_cam.emplace_back(
            static_cast<float>(p_cam.x()), static_cast<float>(p_cam.y()),
            static_cast<float>(p_cam.z()));
    }

    std::vector<cv::Point2f> pts_img;
    cv::projectPoints(pts_cam, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, D, pts_img);
    if (pts_img.size() != 4) {
        return std::nullopt;
    }

    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    for (const auto& p : pts_img) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            return std::nullopt;
        }
        min_x = std::min(min_x, p.x);
        min_y = std::min(min_y, p.y);
        max_x = std::max(max_x, p.x);
        max_y = std::max(max_y, p.y);
    }
    if (!(max_x > min_x && max_y > min_y)) {
        return std::nullopt;
    }
    return cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y);
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
    model_path_     = resolve_pkg_url(mp);
    debug_          = this->declare_parameter<bool>("debug", true);
    detect_color_   = this->declare_parameter<std::string>("detect_color", "RED"); // RED/BLUE
    odom_frame_     = this->declare_parameter<std::string>("target_frame", "odom");
    camera_frame_   = this->declare_parameter<std::string>("camera_frame", camera_frame_);
    use_ba_         = this->declare_parameter<bool>("use_ba", true);
    use_roi_        = this->declare_parameter<bool>("use_roi", use_roi_);
    roi_timeout_s_  = this->declare_parameter<double>("roi_timeout_s", roi_timeout_s_);
    bbox_timeout_s_ = this->declare_parameter<double>("bbox_timeout_s", bbox_timeout_s_);
    const bool use_gpu =
        this->declare_parameter<bool>("use_gpu", false); // true: GPU, false: CPU (default)
    enable_multi_thread_ =
        this->declare_parameter<bool>("enable_multi_thread", false); // OpenVINO throughput mode
    device_name_ = use_gpu ? "GPU" : "CPU";

    img_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    other_callback_group_ = this->get_node_base_interface()->get_default_callback_group();

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

    rclcpp::SubscriptionOptions img_sub_options;
    img_sub_options.callback_group = img_callback_group_;
    rclcpp::SubscriptionOptions other_sub_options;
    other_sub_options.callback_group = other_callback_group_;

    // —— 相机内参（一次性） ——
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
            {
                std::lock_guard<std::mutex> lock(cam_info_mutex_);
                cam_center_ = cv::Point2f(
                    static_cast<float>(camera_info->k[2]), static_cast<float>(camera_info->k[5]));
                cam_info_   = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                pnp_solver_ = std::make_unique<PnPSolver>(cam_info_->k, cam_info_->d);
            }
            cam_info_sub_.reset();
            RCLCPP_INFO(
                this->get_logger(), "Camera info received. fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
                camera_info->k[0], camera_info->k[4], camera_info->k[2], camera_info->k[5]);
        },
        other_sub_options);

    // —— 图像订阅 ——
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorOVNode::imageCallback, this, std::placeholders::_1),
        img_sub_options);

    // —— 订阅 armor_solver/target，用于生成下一帧 ROI ——
    target_sub_ = this->create_subscription<rm_interfaces::msg::Target>(
        "/armor_solver/target", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorOVNode::targetCallback, this, std::placeholders::_1),
        other_sub_options);

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

    bool debug_draw = false;
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        debug_draw = debug_;
    }

    {
        std::lock_guard<std::mutex> lock(roi_mutex_);
        last_image_size_ = img.size();
    }

    cv::Rect roi;
    if (use_roi_) {
        std::lock_guard<std::mutex> lock(roi_mutex_);
        if (next_roi_.has_value()) {
            const rclcpp::Time img_time(img_msg->header.stamp, RCL_ROS_TIME);
            const double age = (img_time - next_roi_stamp_).seconds();
            if (age >= 0.0 && age <= roi_timeout_s_) {
                roi = *next_roi_;
            }
        }
    }
    const cv::Rect full_rc(0, 0, img.cols, img.rows);
    if (roi.width <= 0 || roi.height <= 0) {
        roi = full_rc;
    } else {
        roi &= full_rc;
        if (roi.width <= 0 || roi.height <= 0) {
            roi = full_rc;
        }
    }

    std::vector<ArmorObject> dets;
    cv::Mat roi_img = img(roi);
    if (auto* armor_ov_tup = dynamic_cast<OVArmorTUP*>(model_.get())) {
        armor_ov_tup->detect(roi_img, dets);
    } else if (auto* armor_ov_at = dynamic_cast<OVArmorAT*>(model_.get())) {
        armor_ov_at->detect(roi_img, dets);
    } else {
        (void)model_->run(roi_img);
    }

    // ROI 偏移回原图坐标
    if (roi.x != 0 || roi.y != 0) {
        for (auto& d : dets) {
            d.rect.x += static_cast<float>(roi.x);
            d.rect.y += static_cast<float>(roi.y);
            for (auto& p : d.apex) {
                p.x += static_cast<float>(roi.x);
                p.y += static_cast<float>(roi.y);
            }
        }
    }

    if (debug_draw) {
        cv::Mat draw = img.clone();
        if (dets.size() > 20) {
            RCLCPP_WARN(get_logger(), "Too many armors detected: %d", (int)dets.size());
            dets.clear();
        }
        drawResults(draw, dets, roi);
        result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "bgr8", draw).toImageMsg());
    }
    // filteredArmors(dets);
    // 记录上一帧“目标” bbox（用于 target 回调生成 ROI）
    if (!dets.empty()) {
        cv::Point2f cc(static_cast<float>(img.cols) * 0.5f, static_cast<float>(img.rows) * 0.5f);
        {
            std::lock_guard<std::mutex> lock(cam_info_mutex_);
            if (cam_info_) {
                cc = cam_center_;
            }
        }
        auto best_it =
            std::min_element(dets.begin(), dets.end(), [&](const auto& a, const auto& b) {
                const cv::Point2f ca(
                    a.rect.x + a.rect.width * 0.5f, a.rect.y + a.rect.height * 0.5f);
                const cv::Point2f cb(
                    b.rect.x + b.rect.width * 0.5f, b.rect.y + b.rect.height * 0.5f);
                return cv::norm(ca - cc) < cv::norm(cb - cc);
            });
        cv::Rect bb(
            cvRound(best_it->rect.x), cvRound(best_it->rect.y), cvRound(best_it->rect.width),
            cvRound(best_it->rect.height));
        bb &= full_rc;
        if (bb.width > 0 && bb.height > 0) {
            std::lock_guard<std::mutex> lock(roi_mutex_);
            last_bbox_       = bb;
            last_bbox_stamp_ = rclcpp::Time(img_msg->header.stamp, RCL_ROS_TIME);
        }
    }

    auto armors   = handleDets(dets);
    armors.header = img_msg->header;
    armors_pub_->publish(armors);
    publishMarkers(armors);
}

void ArmorDetectorOVNode::targetCallback(rm_interfaces::msg::Target::ConstSharedPtr target_msg) {
    if (!use_roi_) {
        return;
    }
    if (!target_msg) {
        return;
    }
    if (!target_msg->tracking) {
        std::lock_guard<std::mutex> lock(roi_mutex_);
        next_roi_.reset();
        return;
    }
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info;
    {
        std::lock_guard<std::mutex> lock(cam_info_mutex_);
        cam_info = cam_info_;
    }
    if (!cam_info) {
        return;
    }

    const rclcpp::Time stamp(target_msg->header.stamp, RCL_ROS_TIME);

    cv::Size image_size;
    std::optional<cv::Rect> last_bbox;
    rclcpp::Time last_bbox_stamp(0, 0, RCL_ROS_TIME);
    {
        std::lock_guard<std::mutex> lock(roi_mutex_);
        image_size      = last_image_size_;
        last_bbox       = last_bbox_;
        last_bbox_stamp = last_bbox_stamp_;
    }
    if (image_size.width <= 0 || image_size.height <= 0) {
        return;
    }

    tf2::Transform target_to_cam;
    try {
        const auto tf_stamped = tf2_buffer_->lookupTransform(
            camera_frame_, target_msg->header.frame_id, stamp,
            rclcpp::Duration::from_seconds(0.01));
        tf2::fromMsg(tf_stamped.transform, target_to_cam);
    } catch (...) {
        return;
    }

    const cv::Mat K = cameraMatrixFromInfo(*cam_info);
    const cv::Mat D = distCoeffsFromInfo(*cam_info);

    const auto corners_target = buildArmorCornersTargetFrame(*target_msg);
    if (corners_target.empty()) {
        return;
    }

    bool has_union = false;
    cv::Rect2f roi_rect;
    for (const auto& corners : corners_target) {
        const auto proj = projectArmorToImageRect(corners, target_to_cam, K, D);
        if (!proj.has_value()) {
            continue;
        }
        if (!has_union) {
            roi_rect  = *proj;
            has_union = true;
        } else {
            roi_rect = unionRect(roi_rect, *proj);
        }
    }
    if (!has_union) {
        return;
    }

    if (last_bbox.has_value()) {
        const double age = (stamp - last_bbox_stamp).seconds();
        if (age >= 0.0 && age <= bbox_timeout_s_) {
            roi_rect = unionRect(roi_rect, cv::Rect2f(*last_bbox));
        }
    }

    const int min_side = modelInputSize(model_.get());
    cv::Rect roi       = makeSquareRoi(roi_rect, min_side, image_size);
    if (roi.width <= 0 || roi.height <= 0) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(roi_mutex_);
        next_roi_       = roi;
        next_roi_stamp_ = stamp;
    }
}

void ArmorDetectorOVNode::filteredArmors(std::vector<ArmorObject>& armors) {
    std::string detect_color;
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        detect_color = detect_color_;
    }

    if (detect_color == "RED") {
        armors.erase(
            std::remove_if(
                armors.begin(), armors.end(), [](const ArmorObject& a) { return a.color != 1; }),
            armors.end());
    } else if (detect_color == "BLUE") {
        armors.erase(
            std::remove_if(
                armors.begin(), armors.end(), [](const ArmorObject& a) { return a.color != 0; }),
            armors.end());
    } else if (detect_color == "ALL") {
        // nothing todo
    } else {
        RCLCPP_WARN(get_logger(), "Unknown detect_color_: %s", detect_color.c_str());
    }
}

rm_interfaces::msg::Armors ArmorDetectorOVNode::handleDets(std::vector<ArmorObject>& armors) {
    rm_interfaces::msg::Armors armors_msg;
    rm_interfaces::msg::Armor armor_msg;
    if (armors.size() > 20) {
        RCLCPP_WARN(get_logger(), "Too many armors detected: %d", (int)armors.size());
        return armors_msg;
    }
    PnPSolver* pnp_solver = nullptr;
    {
        std::lock_guard<std::mutex> lock(cam_info_mutex_);
        pnp_solver = pnp_solver_.get();
    }
    if (!pnp_solver) {
        return armors_msg;
    }
    cv::Mat rvec, tvec;
    for (const auto& a : armors) {
        if (use_ba_) {
            if (!pnp_solver->solvePnPWithBA(a, imu_to_camera_, rvec, tvec)) {
                RCLCPP_ERROR(
                    get_logger(), "PnP failed for armor cls = %d, check armor model.", a.cls);
            }
        } else {
            if (!pnp_solver->solvePnP(a, rvec, tvec)) {
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
        armor_msg.distance_to_image_center = pnp_solver->calculateDistanceToCenter(
            cv::Point2f(a.rect.x + a.rect.width * 0.5f, a.rect.y + a.rect.height * 0.5f));
        armors_msg.armors.emplace_back(armor_msg);
    }
    return armors_msg;
}

// ============ 动态参数 ============
rcl_interfaces::msg::SetParametersResult
    ArmorDetectorOVNode::onSetParameters(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    std::lock_guard<std::mutex> lock(param_mutex_);
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
