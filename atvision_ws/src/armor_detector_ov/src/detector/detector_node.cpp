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
#include <string>

// 后端
#include "armor_detector_ov/nn_backend.hpp"
#include "armor_detector_ov/traditional_backend.hpp"

// 模型注册
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
    constexpr double SMALL_W = 135.0 / 1000.0;
    constexpr double SMALL_H = 55.0 / 1000.0;
    constexpr double LARGE_W = 230.0 / 1000.0;
    constexpr double LARGE_H = 55.0 / 1000.0;

    const bool large    = isLargeTarget(target);
    const double half_w = (large ? LARGE_W : SMALL_W) * 0.5;
    const double half_h = (large ? LARGE_H : SMALL_H) * 0.5;

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
        } else {
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

    // —— 后端类型 ——
    backend_type_ = this->declare_parameter<std::string>("backend_type", "nn");
    RCLCPP_INFO(get_logger(), "Backend type: %s", backend_type_.c_str());

    // —— 通用参数 ——
    debug_           = this->declare_parameter<bool>("debug", true);
    detect_color_    = this->declare_parameter<std::string>("detect_color", "RED");
    odom_frame_      = this->declare_parameter<std::string>("target_frame", "odom");
    camera_frame_    = this->declare_parameter<std::string>("camera_frame", camera_frame_);
    use_ba_          = this->declare_parameter<bool>("use_ba", true);
    draw_latency_    = this->declare_parameter<bool>("draw_latency", draw_latency_);

    // —— NN后端参数 ——
    model_name_   = this->declare_parameter<std::string>("model_name", "armor_tup");
    const auto mp = this->declare_parameter<std::string>(
        "model_path", "package://armor_detector_ov/model/armor_tup.xml");
    model_path_      = resolve_pkg_url(mp);
    device_name_ = this->declare_parameter<std::string>("device_name", device_name_);
    device_priorities_ =
        this->declare_parameter<std::string>("device_priorities", device_priorities_);
    enable_multi_thread_ = this->declare_parameter<bool>("enable_multi_thread", false);
    pipeline_queue_size_ =
        this->declare_parameter<int>("pipeline_queue_size", pipeline_queue_size_);
    pipeline_num_requests_ =
        this->declare_parameter<int>("pipeline_num_requests", pipeline_num_requests_);

    // —— ROI参数（仅NN后端使用） ——
    use_roi_         = this->declare_parameter<bool>("use_roi", use_roi_);
    roi_timeout_s_   = this->declare_parameter<double>("roi_timeout_s", roi_timeout_s_);
    bbox_timeout_s_  = this->declare_parameter<double>("bbox_timeout_s", bbox_timeout_s_);
    roi_future_tolerance_s_ =
        this->declare_parameter<double>("roi_future_tolerance_s", roi_future_tolerance_s_);
    roi_disable_dist_m_ = this->declare_parameter<double>("roi_disable_dist_m", roi_disable_dist_m_);
    roi_scale_          = this->declare_parameter<double>("roi_scale", roi_scale_);
    roi_smooth_alpha_   = this->declare_parameter<double>("roi_smooth_alpha", roi_smooth_alpha_);
    roi_clear_on_miss_  = this->declare_parameter<bool>("roi_clear_on_miss", roi_clear_on_miss_);
    roi_miss_disable_s_ =
        this->declare_parameter<double>("roi_miss_disable_s", roi_miss_disable_s_);
    drop_out_of_order_  = this->declare_parameter<bool>("drop_out_of_order", drop_out_of_order_);

    // —— 回调组 ——
    img_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    other_callback_group_ = this->get_node_base_interface()->get_default_callback_group();

    // —— 创建后端 ——
    detector_ = createBackend();
    if (!detector_ || !detector_->isInitialized()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize detector backend");
    } else {
        // 设置检测颜色
        const int color_int = (detect_color_ == "RED") ? 1 : 0;
        detector_->setDetectColor(color_int);
        RCLCPP_INFO(get_logger(), "Detector backend initialized: %s",
                    detector_->getBackendName().c_str());
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

std::unique_ptr<DetectorBackend> ArmorDetectorOVNode::createBackend() {
    if (backend_type_ == "traditional") {
        // 创建传统后端
        const int binary_thres = this->declare_parameter<int>("binary_thres", 160);

        TraditionalBackend::LightParams light_params;
        light_params.min_ratio = this->declare_parameter<double>("light.min_ratio", 0.1);
        light_params.max_ratio = this->declare_parameter<double>("light.max_ratio", 0.55);
        light_params.max_angle = this->declare_parameter<double>("light.max_angle", 45.0);
        light_params.color_diff_thresh =
            this->declare_parameter<int>("light.color_diff_thresh", 50);

        TraditionalBackend::ArmorParams armor_params;
        armor_params.min_light_ratio =
            this->declare_parameter<double>("armor.min_light_ratio", 0.7);
        armor_params.min_small_center_distance =
            this->declare_parameter<double>("armor.min_small_center_distance", 0.8);
        armor_params.max_small_center_distance =
            this->declare_parameter<double>("armor.max_small_center_distance", 3.2);
        armor_params.min_large_center_distance =
            this->declare_parameter<double>("armor.min_large_center_distance", 3.2);
        armor_params.max_large_center_distance =
            this->declare_parameter<double>("armor.max_large_center_distance", 5.5);
        armor_params.max_angle = this->declare_parameter<double>("armor.max_angle", 35.0);

        auto backend =
            std::make_unique<TraditionalBackend>(binary_thres, light_params, armor_params);

        // 初始化数字分类器（可选）
        const auto classifier_model = this->declare_parameter<std::string>(
            "classifier.model_path", "package://armor_detector_ov/model/mlp.onnx");
        const auto classifier_label = this->declare_parameter<std::string>(
            "classifier.label_path", "package://armor_detector_ov/model/label.txt");
        const double classifier_threshold =
            this->declare_parameter<double>("classifier.threshold", 0.7);

        const std::string resolved_model = resolve_pkg_url(classifier_model);
        const std::string resolved_label = resolve_pkg_url(classifier_label);

        if (std::filesystem::exists(resolved_model) && std::filesystem::exists(resolved_label)) {
            backend->initClassifier(
                resolved_model, resolved_label, classifier_threshold, {"negative"}, false);
            RCLCPP_INFO(
                get_logger(), "Traditional backend with classifier: %s",
                resolved_model.c_str());
        } else {
            RCLCPP_WARN(
                get_logger(),
                "Traditional backend without classifier (model/label not found)");
        }

        // 传统后端不使用ROI
        use_roi_ = false;

        return backend;
    } else {
        // 创建NN后端
        NNBackend::Config config;
        config.model_name = model_name_;
        config.model_path = model_path_;
        config.device = device_name_;
        config.device_priorities = device_priorities_;
        config.enable_multi_thread = enable_multi_thread_;
        config.queue_size = pipeline_queue_size_;
        config.num_requests = pipeline_num_requests_;

        auto backend = std::make_unique<NNBackend>(config);
        if (!backend->init()) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize NN backend");
            return nullptr;
        }

        // 打印执行设备
        const auto exec_devs = backend->executionDevices();
        std::string exec_join;
        for (size_t i = 0; i < exec_devs.size(); ++i) {
            if (i) exec_join += ", ";
            exec_join += exec_devs[i];
        }
        RCLCPP_INFO(get_logger(), "NN backend devices: %s",
                    exec_join.empty() ? device_name_.c_str() : exec_join.c_str());

        return backend;
    }
}

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
    if (!img_msg || !detector_ || !detector_->isInitialized()) {
        return;
    }

    const rclcpp::Time img_stamp(img_msg->header.stamp, RCL_ROS_TIME);
    const std::string frame_id = img_msg->header.frame_id;

    bool debug_draw = false;
    {
        std::lock_guard<std::mutex> lock(param_mutex_);
        debug_draw = debug_;
    }

    const int w = static_cast<int>(img_msg->width);
    const int h = static_cast<int>(img_msg->height);
    if (w <= 0 || h <= 0) {
        RCLCPP_WARN(get_logger(), "Invalid image size: %dx%d", w, h);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(roi_mutex_);
        last_image_size_ = cv::Size(w, h);
    }

    // 计算ROI（仅NN后端且支持ROI时使用）
    cv::Rect roi;
    if (use_roi_ && detector_->supportsROI()) {
        std::lock_guard<std::mutex> lock(roi_mutex_);
        if (img_stamp < roi_disable_until_) {
            // ROI 临时禁用窗口内
        } else if (next_roi_.has_value()) {
            const double age = (img_stamp - next_roi_stamp_).seconds();
            if (age >= -roi_future_tolerance_s_ && age <= roi_timeout_s_) {
                roi = *next_roi_;
            }
        }
    }

    const cv::Rect full_rc(0, 0, w, h);
    if (roi.width <= 0 || roi.height <= 0) {
        roi = full_rc;
    } else {
        roi &= full_rc;
        if (roi.width <= 0 || roi.height <= 0) {
            roi = full_rc;
        }
    }

    // 根据后端能力选择同步或异步路径
    if (detector_->supportsAsync()) {
        // 异步模式：enqueue + tryGetResult
        uint64_t enqueued_frame_id = detector_->enqueue(
            cv_bridge::toCvShare(img_msg, "bgr8")->image,
            roi, img_stamp);

        if (enqueued_frame_id > 0) {
            // 保存时间戳和frame_id映射
            std::lock_guard<std::mutex> lk(stamp_map_mutex_);
            frame_stamp_map_[enqueued_frame_id] = {img_stamp, frame_id};
        }

        // 尝试获取之前帧的结果
        uint64_t result_frame_id;
        rclcpp::Time result_stamp;
        std::vector<ArmorObject> dets;
        while (detector_->tryGetResult(result_frame_id, result_stamp, dets)) {
            // 使用结果中的stamp发布
            publishArmors(dets, result_stamp, frame_id);
        }
    } else {
        // 同步模式
        if (drop_out_of_order_ && last_sync_pub_stamp_.nanoseconds() != 0
            && img_stamp < last_sync_pub_stamp_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Drop out-of-order image: stamp=%.6f last=%.6f",
                img_stamp.seconds(), last_sync_pub_stamp_.seconds());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
            return;
        }
        if (!cv_ptr || cv_ptr->image.empty()) {
            RCLCPP_WARN(get_logger(), "Image empty");
            return;
        }

        cv::Mat img = cv_ptr->image;
        std::vector<ArmorObject> dets = detector_->detect(img, roi);

        // Debug绘制
        if (debug_draw) {
            cv::Mat draw = detector_->getDebugImage();
            if (draw.empty()) {
                draw = img.clone();
                drawResults(draw, dets, roi);
            }
            result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "bgr8", draw).toImageMsg());
        }

        // 记录bbox
        if (!dets.empty()) {
            cv::Point2f cc(static_cast<float>(img.cols) * 0.5f, static_cast<float>(img.rows) * 0.5f);
            {
                std::lock_guard<std::mutex> lock(cam_info_mutex_);
                if (cam_info_) cc = cam_center_;
            }
            auto best_it =
                std::min_element(dets.begin(), dets.end(), [&](const auto& a, const auto& b) {
                    const cv::Point2f ca(a.rect.x + a.rect.width * 0.5f, a.rect.y + a.rect.height * 0.5f);
                    const cv::Point2f cb(b.rect.x + b.rect.width * 0.5f, b.rect.y + b.rect.height * 0.5f);
                    return cv::norm(ca - cc) < cv::norm(cb - cc);
                });
            cv::Rect bb(cvRound(best_it->rect.x), cvRound(best_it->rect.y),
                        cvRound(best_it->rect.width), cvRound(best_it->rect.height));
            bb &= full_rc;
            if (bb.width > 0 && bb.height > 0) {
                std::lock_guard<std::mutex> lock(roi_mutex_);
                last_bbox_       = bb;
                last_bbox_stamp_ = img_stamp;
            }
        }

        if (roi_clear_on_miss_ && dets.empty() && roi != full_rc) {
            std::lock_guard<std::mutex> lock(roi_mutex_);
            next_roi_.reset();
            if (roi_miss_disable_s_ > 0.0) {
                roi_disable_until_ = img_stamp + rclcpp::Duration::from_seconds(roi_miss_disable_s_);
            }
        }

        publishArmors(dets, img_stamp, frame_id);

        if (drop_out_of_order_) {
            last_sync_pub_stamp_ = img_stamp;
        }
    }
}

void ArmorDetectorOVNode::publishArmors(std::vector<ArmorObject>& armors,
                                        const rclcpp::Time& stamp,
                                        const std::string& frame_id) {
    Eigen::Matrix3d imu_to_camera = Eigen::Matrix3d::Identity();
    bool tf_ok = false;
    try {
        auto odom_to_gimbal = tf2_buffer_->lookupTransform(
            odom_frame_, frame_id, stamp, rclcpp::Duration::from_seconds(0.01));
        auto msg_q = odom_to_gimbal.transform.rotation;
        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3 tf2_matrix = tf2::Matrix3x3(tf_q);
        imu_to_camera << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1], tf2_matrix.getRow(0)[2],
            tf2_matrix.getRow(1)[0], tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
            tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1], tf2_matrix.getRow(2)[2];
        tf_ok = true;
    } catch (...) {
        tf_ok = false;
    }

    auto armors_msg = handleDets(armors, tf_ok ? &imu_to_camera : nullptr);
    armors_msg.header.stamp = stamp;
    armors_msg.header.frame_id = frame_id;
    armors_pub_->publish(armors_msg);
    publishMarkers(armors_msg);
}

void ArmorDetectorOVNode::targetCallback(rm_interfaces::msg::Target::ConstSharedPtr target_msg) {
    if (!use_roi_ || !detector_ || !detector_->supportsROI()) {
        return;
    }
    if (!target_msg || !target_msg->tracking) {
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

    // 距离过近禁用ROI
    {
        const tf2::Vector3 p_tgt(
            target_msg->position.x, target_msg->position.y, target_msg->position.z);
        const tf2::Vector3 p_cam = target_to_cam * p_tgt;
        const double dist_m =
            std::sqrt(p_cam.x() * p_cam.x() + p_cam.y() * p_cam.y() + p_cam.z() * p_cam.z());
        if (std::isfinite(dist_m) && dist_m > 0.0 && dist_m < roi_disable_dist_m_) {
            std::lock_guard<std::mutex> lock(roi_mutex_);
            next_roi_.reset();
            return;
        }
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
        if (!proj.has_value()) continue;
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

    // 放大ROI
    {
        const float s = std::clamp(static_cast<float>(roi_scale_), 1.0f, 4.0f);
        if (s > 1.0f) {
            const float cx = roi_rect.x + roi_rect.width * 0.5f;
            const float cy = roi_rect.y + roi_rect.height * 0.5f;
            const float w  = roi_rect.width * s;
            const float h  = roi_rect.height * s;
            roi_rect       = cv::Rect2f(cx - w * 0.5f, cy - h * 0.5f, w, h);
        }
    }

    // 模型输入尺寸作为最小边长
    constexpr int DEFAULT_INPUT_SIZE = 416;
    cv::Rect roi = makeSquareRoi(roi_rect, DEFAULT_INPUT_SIZE, image_size);
    if (roi.width <= 0 || roi.height <= 0) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(roi_mutex_);
        const float a = std::clamp(static_cast<float>(roi_smooth_alpha_), 0.0f, 1.0f);
        if (next_roi_.has_value() && a > 0.0f && a < 1.0f) {
            const cv::Rect prev = *next_roi_;
            const float pcx = prev.x + prev.width * 0.5f;
            const float pcy = prev.y + prev.height * 0.5f;
            const float ncx = roi.x + roi.width * 0.5f;
            const float ncy = roi.y + roi.height * 0.5f;
            const float cx  = pcx * (1.0f - a) + ncx * a;
            const float cy  = pcy * (1.0f - a) + ncy * a;

            float side = prev.width * (1.0f - a) + roi.width * a;
            side       = std::max(side, static_cast<float>(roi.width));
            const int max_side = std::min(image_size.width, image_size.height);
            int iside          = std::clamp(static_cast<int>(std::round(side)), 1, max_side);

            int x = static_cast<int>(std::round(cx - iside * 0.5f));
            int y = static_cast<int>(std::round(cy - iside * 0.5f));
            x     = std::clamp(x, 0, image_size.width - iside);
            y     = std::clamp(y, 0, image_size.height - iside);
            next_roi_ = cv::Rect(x, y, iside, iside);
        } else {
            next_roi_ = roi;
        }
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
            std::remove_if(armors.begin(), armors.end(), [](const ArmorObject& a) { return a.color != 1; }),
            armors.end());
    } else if (detect_color == "BLUE") {
        armors.erase(
            std::remove_if(armors.begin(), armors.end(), [](const ArmorObject& a) { return a.color != 0; }),
            armors.end());
    }
}

rm_interfaces::msg::Armors ArmorDetectorOVNode::handleDets(
    std::vector<ArmorObject>& armors, const Eigen::Matrix3d* imu_to_camera) {
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
        if (use_ba_ && imu_to_camera) {
            if (!pnp_solver->solvePnPWithBA(a, *imu_to_camera, rvec, tvec)) {
                RCLCPP_ERROR(get_logger(), "PnP failed for armor cls = %d", a.cls);
            }
        } else {
            if (!pnp_solver->solvePnP(a, rvec, tvec)) {
                RCLCPP_ERROR(get_logger(), "PnP failed for armor cls=%d", a.cls);
            }
        }
        cv::Mat Rcv;
        cv::Rodrigues(rvec, Rcv);
        tf2::Matrix3x3 R(
            Rcv.at<double>(0, 0), Rcv.at<double>(0, 1), Rcv.at<double>(0, 2),
            Rcv.at<double>(1, 0), Rcv.at<double>(1, 1), Rcv.at<double>(1, 2),
            Rcv.at<double>(2, 0), Rcv.at<double>(2, 1), Rcv.at<double>(2, 2));
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
                if (detector_) {
                    detector_->setDetectColor(v == "RED" ? 1 : 0);
                }
                RCLCPP_INFO(get_logger(), "detect_color = %s", detect_color_.c_str());
            } else {
                res.successful = false;
                res.reason     = "detect_color must be RED/BLUE";
            }
        }
    }
    return res;
}

void ArmorDetectorOVNode::publishMarkers(const rm_interfaces::msg::Armors& armors_msg) noexcept {
    using visualization_msgs::msg::Marker;

    marker_array_.markers.clear();

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

    int id = 0;
    for (const auto& a : armors_msg.armors) {
        Marker marker = armor_marker_;
        marker.header = armors_msg.header;
        marker.id     = id++;
        marker.pose   = a.pose;
        marker_array_.markers.emplace_back(std::move(marker));
    }

    marker_pub_->publish(marker_array_);
}

void ArmorDetectorOVNode::drawResults(
    cv::Mat& src, const std::vector<ArmorObject>& armor_objects, const cv::Rect& roi) noexcept {
    const int img_min = std::max(1, std::min(src.cols, src.rows));
    for (const auto& obj : armor_objects) {
        const cv::Rect img_rc(0, 0, src.cols, src.rows);
        cv::Rect r(cvRound(obj.rect.x), cvRound(obj.rect.y),
                   cvRound(obj.rect.width), cvRound(obj.rect.height));
        r &= img_rc;
        if (r.empty()) continue;

        const cv::Scalar color = boxColor(obj.color);
        const int thick        = std::clamp(img_min / 600, 2, 6);
        const double fscale    = std::clamp(img_min / 900.0, 0.8, 1.5);
        const int fthick       = std::clamp(thick - 1, 1, 3);

        fillRectAlpha(src, r, color, 0.15);

        for (int i = 0; i < 4; ++i)
            cv::line(src, obj.apex[i], obj.apex[(i + 1) % 4], color, thick, cv::LINE_AA);

        std::string label =
            cv::format("%s %d %.2f%%", color_letter_(obj.color).c_str(), obj.cls, obj.prob * 100.0);

        int base = 0;
        cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, fscale, fthick, &base);
        int pad = 6;
        cv::Point tl(r.x, std::max(0, r.y - ts.height - base - 2 * pad));
        cv::Rect lb(tl.x, tl.y, ts.width + 2 * pad, ts.height + base + 2 * pad);
        lb &= cv::Rect(0, 0, src.cols, src.rows);
        if (lb.width > 0 && lb.height > 0) {
            cv::Mat roi_mat = src(lb);
            cv::Mat bg(roi_mat.size(), roi_mat.type(), color);
            cv::addWeighted(bg, 0.8, roi_mat, 0.2, 0.0, roi_mat);
            cv::putText(src, label, {lb.x + pad, lb.y + pad + ts.height},
                        cv::FONT_HERSHEY_SIMPLEX, fscale, {0, 0, 0}, fthick + 2, cv::LINE_AA);
            cv::putText(src, label, {lb.x + pad, lb.y + pad + ts.height},
                        cv::FONT_HERSHEY_SIMPLEX, fscale, {255, 255, 255}, fthick, cv::LINE_AA);
        }
    }
    cv::rectangle(src, roi, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

} // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorOVNode)
