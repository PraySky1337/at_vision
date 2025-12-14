#include "rm_calibration/calibration_node.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <limits>
#include <sstream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace camera_imu_calibration {

CalibrationNode::CalibrationNode()
    : Node("calibration_node")
    , intrinsic_calibrated_(false) {
    // Declare and load parameters
    declareParameters();
    loadParameters();

    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this, camera_name_, camera_info_url_);
    camera_info_        = camera_info_manager_->getCameraInfo();
    camera_info_loaded_ = camera_info_manager_->isCalibrated();

    // Initialize TF2
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Setup calibrators based on mode
    if (mode_ == CalibrationMode::INTRINSIC) {
        setupIntrinsicMode();
    } else {
        setupExtrinsicMode();
    }

    // Callback groups to allow services and subscriptions to run concurrently.
    // Use a mutually-exclusive group for image/camera_info to preserve ordering and avoid
    // concurrent access to shared state (timestamps, previous corners, etc.).
    image_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscribe to image and camera info topics
    rclcpp::SubscriptionOptions image_opts;
    image_opts.callback_group = image_callback_group_;
    image_sub_                = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&CalibrationNode::imageCallback, this, std::placeholders::_1), image_opts);

    rclcpp::SubscriptionOptions info_opts;
    info_opts.callback_group = image_callback_group_;
    camera_info_sub_         = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&CalibrationNode::cameraInfoCallback, this, std::placeholders::_1), info_opts);

    // Create services
    calibrate_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/calibrate",
        std::bind(
            &CalibrationNode::calibrateServiceCallback, this, std::placeholders::_1,
            std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);

    capture_sample_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/capture_sample",
        std::bind(
            &CalibrationNode::captureSampleServiceCallback, this, std::placeholders::_1,
            std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);

    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reset",
        std::bind(
            &CalibrationNode::resetServiceCallback, this, std::placeholders::_1,
            std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);

    preview_pub_ = image_transport::create_publisher(this, "~/preview");

    RCLCPP_INFO(
        this->get_logger(), "Calibration node initialized in %s mode",
        mode_ == CalibrationMode::INTRINSIC ? "INTRINSIC" : "EXTRINSIC");
}

void CalibrationNode::declareParameters() {
    this->declare_parameter("mode", "intrinsic");
    this->declare_parameter("board_width", 9);
    this->declare_parameter("board_height", 6);
    this->declare_parameter("square_size", 0.025);
    this->declare_parameter("required_frames", 20);
    this->declare_parameter("target_samples", 25);
    this->declare_parameter("quality_threshold", 0.2);
    this->declare_parameter("display_fps", 10.0);
    this->declare_parameter("max_tf_time_diff", 0.3);
    this->declare_parameter("imu_frame", "gimbal_link");
    this->declare_parameter("base_frame", "odom");
    this->declare_parameter("camera_optical_frame", "camera_optical_frame");
    this->declare_parameter("camera_link_frame", "camera_link");
    this->declare_parameter("camera_name", "camera");
    this->declare_parameter("camera_info_url", "");
    this->declare_parameter("camera_info_topic", "/camera_info");
    this->declare_parameter("enable_preview_window", false);
    this->declare_parameter("weight_corner_quality", 0.3);
    this->declare_parameter("weight_board_size", 0.2);
    this->declare_parameter("weight_board_angle", 0.3);
    this->declare_parameter("weight_image_sharpness", 0.2);
    this->declare_parameter("weight_reprojection_error", 0.4);
    this->declare_parameter("weight_imu_diversity", 0.3);
    this->declare_parameter("weight_board_pose", 0.3);
}

void CalibrationNode::loadParameters() {
    std::string mode_str = this->get_parameter("mode").as_string();
    mode_ = (mode_str == "extrinsic") ? CalibrationMode::EXTRINSIC : CalibrationMode::INTRINSIC;

    board_width_           = this->get_parameter("board_width").as_int();
    board_height_          = this->get_parameter("board_height").as_int();
    square_size_           = this->get_parameter("square_size").as_double();
    target_samples_        = this->get_parameter("target_samples").as_int();
    quality_threshold_     = this->get_parameter("quality_threshold").as_double();
    display_fps_           = this->get_parameter("display_fps").as_double();
    max_tf_time_diff_      = this->get_parameter("max_tf_time_diff").as_double();
    imu_frame_             = this->get_parameter("imu_frame").as_string();
    base_frame_            = this->get_parameter("base_frame").as_string();
    camera_optical_frame_  = this->get_parameter("camera_optical_frame").as_string();
    camera_link_frame_     = this->get_parameter("camera_link_frame").as_string();
    camera_name_           = this->get_parameter("camera_name").as_string();
    camera_info_url_       = this->get_parameter("camera_info_url").as_string();
    camera_info_topic_     = this->get_parameter("camera_info_topic").as_string();
    enable_preview_window_ = this->get_parameter("enable_preview_window").as_bool();

    // Backward compatibility with required_frames
    int legacy_required = this->get_parameter("required_frames").as_int();
    if (target_samples_ <= 0) {
        target_samples_ = legacy_required;
    }

    target_samples_    = std::max(1, target_samples_);
    quality_threshold_ = std::max(0.0, std::min(1.0, quality_threshold_));
    display_fps_       = std::max(0.0, display_fps_);
    max_tf_time_diff_  = std::max(0.0, max_tf_time_diff_);

    loadQualityWeights();
}

void CalibrationNode::setupIntrinsicMode() {
    intrinsic_calibrator_ = std::make_unique<IntrinsicCalibrator>(
        board_width_, board_height_, square_size_, target_samples_);
    intrinsic_calibrator_->configureCollection(target_samples_, quality_threshold_);

    RCLCPP_INFO(
        this->get_logger(),
        "Intrinsic calibration setup: %dx%d board, %.3f m squares, %d target frames", board_width_,
        board_height_, square_size_, target_samples_);
}

void CalibrationNode::setupExtrinsicMode() {
    if (extrinsic_calibrator_) {
        return;
    }

    // Refresh from manager if we don't have a camera_info yet
    if (!camera_info_loaded_) {
        camera_info_        = camera_info_manager_->getCameraInfo();
        camera_info_loaded_ = camera_info_manager_->isCalibrated();
    }

    if (!camera_info_loaded_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Camera info not available. Provide a camera_info_url or publish CameraInfo on %s",
            camera_info_topic_.c_str());
        return;
    }

    if (!loadIntrinsicsFromCameraInfo(camera_info_)) {
        RCLCPP_ERROR(
            this->get_logger(), "Invalid CameraInfo; cannot initialize extrinsic calibrator");
        return;
    }

    // Load camera intrinsics
    intrinsic_calibrated_ = true;

    extrinsic_calibrator_ = std::make_unique<ExtrinsicCalibrator>(
        board_width_, board_height_, square_size_, camera_matrix_, dist_coeffs_, target_samples_);

    extrinsic_calibrator_->configureCollection(target_samples_, quality_threshold_);

    RCLCPP_INFO(
        this->get_logger(),
        "Extrinsic calibration setup: %dx%d board, %.3f m squares, %d target samples", board_width_,
        board_height_, square_size_, target_samples_);
}

void CalibrationNode::refreshCollectionParameters() {
    target_samples_    = this->get_parameter("target_samples").as_int();
    quality_threshold_ = this->get_parameter("quality_threshold").as_double();
    display_fps_       = this->get_parameter("display_fps").as_double();

    target_samples_    = std::max(1, target_samples_);
    quality_threshold_ = std::max(0.0, std::min(1.0, quality_threshold_));
    display_fps_       = std::max(0.0, display_fps_);

    loadQualityWeights();

    if (mode_ == CalibrationMode::INTRINSIC) {
        if (intrinsic_calibrator_) {
            intrinsic_calibrator_->configureCollection(target_samples_, quality_threshold_);
        }
    } else if (extrinsic_calibrator_) {
        extrinsic_calibrator_->configureCollection(target_samples_, quality_threshold_);
    }
}

void CalibrationNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    last_image_receive_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count(),
        std::memory_order_relaxed);

    const rclcpp::Time msg_stamp(msg->header.stamp, this->get_clock()->get_clock_type());
    if (last_image_stamp_.nanoseconds() != 0 && msg_stamp < last_image_stamp_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Non-monotonic image header stamp detected (prev=%.6f, curr=%.6f). "
            "This usually indicates multiple publishers on /image_raw or a time source jump.",
            last_image_stamp_.seconds(), msg_stamp.seconds());
    }
    last_image_stamp_ = msg_stamp;

    const bool capture_pending     = capture_requested_.load();
    const bool has_preview_subs    = preview_pub_ && (preview_pub_.getNumSubscribers() > 0);
    const bool want_preview_output = enable_preview_window_ || has_preview_subs;
    if (!capture_pending && !want_preview_output) {
        // Nothing to do unless a capture request is waiting.
        return;
    }

    // Throttle expensive processing + preview publishing. Otherwise, running chessboard detection
    // on every incoming frame can starve the executor and make the preview appear "stuck" when
    // the board isn't visible.
    bool publish_visual = want_preview_output;
    if (want_preview_output && display_fps_ > 0.0 && !capture_pending) {
        rclcpp::Time now = this->now();
        double dt        = (last_display_time_.nanoseconds() == 0) ? (1.0 / display_fps_)
                                                                   : (now - last_display_time_).seconds();
        if (dt >= (1.0 / display_fps_)) {
            last_display_time_ = now;
        } else {
            return;
        }
    }

    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat display_image;
    double current_quality = 0.0;
    // (last_image_stamp_ already updated above)

    if (mode_ == CalibrationMode::INTRINSIC) {
        display_image = cv_ptr->image;
        std::vector<cv::Point2f> corners;
        bool found = intrinsic_calibrator_->detectCorners(cv_ptr->image, corners, display_image);

        if (found) {
            current_quality = quality_scorer_.scoreIntrinsicFrame(
                cv_ptr->image, corners, cv::Size(board_width_, board_height_), previous_corners_);
            previous_corners_ = corners;
        }

        handleCaptureRequestIntrinsic(found, corners, cv_ptr->image.size(), current_quality);
    } else {
        Eigen::Matrix3d R_imu;
        Eigen::Vector3d t_imu = Eigen::Vector3d::Zero();
        bool imu_ready        = getIMUPose(msg->header.stamp, R_imu, t_imu);

        std::vector<cv::Point2f> corners;
        cv::Mat rvec, tvec;
        double reprojection_error = 0.0;
        bool found                = false;

        if (extrinsic_calibrator_ && imu_ready) {
            display_image = cv_ptr->image;
            found         = extrinsic_calibrator_->detectSample(
                cv_ptr->image, R_imu, corners, display_image, rvec, tvec, reprojection_error);

            if (found) {
                current_quality = quality_scorer_.scoreExtrinsicSample(
                    cv_ptr->image, corners, rvec, tvec, R_imu,
                    last_accepted_imu_valid_ ? &last_accepted_imu_R_ : nullptr, reprojection_error);
                previous_corners_ = corners;
            }
        } else {
            display_image = cv_ptr->image;
        }

        handleCaptureRequestExtrinsic(
            imu_ready, found, corners, rvec, tvec, R_imu, t_imu, current_quality,
            reprojection_error);
    }

    drawStatusOverlay(display_image, current_quality);

    if (publish_visual && enable_preview_window_) {
        cv::imshow("Calibration", display_image);
        cv::waitKey(1);
    }

    if (publish_visual && has_preview_subs) {
        std_msgs::msg::Header header = msg->header;
        auto preview_msg =
            cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, display_image)
                .toImageMsg();
        preview_pub_.publish(preview_msg);
    }
}

void CalibrationNode::handleCaptureRequestIntrinsic(
    bool found, const std::vector<cv::Point2f>& corners, const cv::Size& image_size,
    double quality) {
    if (!capture_requested_.load()) {
        return;
    }

    std::unique_lock<std::mutex> lock(capture_mutex_);
    if (!capture_requested_) {
        return;
    }

    bool accepted = false;
    if (found) {
        accepted = intrinsic_calibrator_->tryAddSample(corners, image_size, quality);
    }

    if (!found) {
        last_capture_success_ = false;
        last_capture_message_ = "Chessboard not detected in current frame";
    } else if (accepted) {
        last_capture_success_ = true;
        std::ostringstream oss;
        oss << "Sample stored (" << intrinsic_calibrator_->getCollectedFrames() << "/"
            << target_samples_ << ")";
        last_capture_message_ = oss.str();
    } else {
        last_capture_success_ = false;
        std::ostringstream oss;
        oss << "Sample rejected (quality " << std::fixed << std::setprecision(2) << quality << ")";
        last_capture_message_ = oss.str();
    }

    capture_requested_      = false;
    capture_response_ready_ = true;
    lock.unlock();

    {
        std::lock_guard<std::mutex> state_lock(data_mutex_);
        state_ = CalibrationState::IDLE;
    }

    capture_cv_.notify_all();
}

void CalibrationNode::handleCaptureRequestExtrinsic(
    bool imu_ready, bool found, const std::vector<cv::Point2f>& corners, const cv::Mat& rvec,
    const cv::Mat& tvec, const Eigen::Matrix3d& R_imu, const Eigen::Vector3d& t_imu, double quality,
    double reprojection_error) {
    if (!capture_requested_.load()) {
        return;
    }

    std::unique_lock<std::mutex> lock(capture_mutex_);
    if (!capture_requested_) {
        return;
    }

    bool accepted = false;
    if (!imu_ready) {
        last_capture_success_ = false;
        last_capture_message_ = "IMU transform unavailable";
    } else if (!extrinsic_calibrator_) {
        last_capture_success_ = false;
        last_capture_message_ = "Extrinsic calibrator not ready";
    } else if (!found) {
        last_capture_success_ = false;
        last_capture_message_ = "Chessboard not detected in current frame";
    } else {
        accepted = extrinsic_calibrator_->tryAddSample(
            rvec, tvec, R_imu, t_imu, corners, quality, reprojection_error);
        if (accepted) {
            last_accepted_imu_R_     = R_imu;
            last_accepted_imu_valid_ = true;
            std::ostringstream oss;
            oss << "Sample stored (" << extrinsic_calibrator_->getCollectedSamples() << "/"
                << target_samples_ << ")";
            last_capture_success_ = true;
            last_capture_message_ = oss.str();
        } else {
            std::ostringstream oss;
            oss << "Sample rejected (quality " << std::fixed << std::setprecision(2) << quality
                << ")";
            last_capture_success_ = false;
            last_capture_message_ = oss.str();
        }
    }

    capture_requested_      = false;
    capture_response_ready_ = true;
    lock.unlock();

    {
        std::lock_guard<std::mutex> state_lock(data_mutex_);
        state_ = CalibrationState::IDLE;
    }

    capture_cv_.notify_all();
}

bool CalibrationNode::getIMUPose(
    const rclcpp::Time& stamp, Eigen::Matrix3d& R_imu, Eigen::Vector3d& t_imu) {
    // Try to use the timestamped transform; fall back to latest if it's within tolerance.
    geometry_msgs::msg::TransformStamped transform;
    bool used_fallback = false;
    try {
        tf2::TimePoint tf_time = tf2::timeFromSec(stamp.seconds());
        transform              = tf_buffer_->lookupTransform(
            base_frame_, imu_frame_, tf_time, tf2::durationFromSec(0.2));
    } catch (const tf2::ExtrapolationException&) {
        try {
            transform = tf_buffer_->lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
            used_fallback = true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000, "Could not get transform: %s",
                ex.what());
            return false;
        }
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000, "Could not get transform: %s", ex.what());
        return false;
    }

    if (used_fallback) {
        rclcpp::Time tf_stamp(transform.header.stamp);
        double dt = std::abs((tf_stamp - stamp).seconds());
        if (dt > max_tf_time_diff_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "IMU transform too far from image time (|%.0f| ms > %.0f ms)", dt * 1000.0,
                max_tf_time_diff_ * 1000.0);
            return false;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Using latest IMU transform; dt=%.0f ms (image %.3f, tf %.3f)", dt * 1000.0,
            stamp.seconds(), tf_stamp.seconds());
    }

    tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);

    tf2::Matrix3x3 m(q);
    R_imu << m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2];
    t_imu << transform.transform.translation.x, transform.transform.translation.y,
        transform.transform.translation.z;

    return true;
}

void CalibrationNode::calibrateServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (state_ == CalibrationState::CALIBRATING) {
            response->success = false;
            response->message = "Calibration already in progress";
            return;
        }
    }

    if (capture_requested_.load()) {
        response->success = false;
        response->message = "Capture request in progress; try again";
        return;
    }

    if (mode_ == CalibrationMode::EXTRINSIC && !extrinsic_calibrator_) {
        response->success = false;
        response->message = "Extrinsic calibrator not ready (waiting for CameraInfo)";
        return;
    }

    refreshCollectionParameters();

    int collected = 0;
    if (mode_ == CalibrationMode::INTRINSIC) {
        collected = intrinsic_calibrator_->getCollectedFrames();
    } else if (extrinsic_calibrator_) {
        collected = extrinsic_calibrator_->getCollectedSamples();
    }

    if (collected < 3) {
        std::ostringstream oss;
        oss << "Not enough samples to calibrate (" << collected << "/3)";
        response->success = false;
        response->message = oss.str();
        return;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_ = CalibrationState::CALIBRATING;
    }

    triggerCalibration();

    std::ostringstream oss;
    oss << "Started calibration with " << collected << " samples";

    response->success = true;
    response->message = oss.str();

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void CalibrationNode::captureSampleServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (state_ == CalibrationState::CALIBRATING) {
            response->success = false;
            response->message = "Calibration is running; wait until it finishes";
            return;
        }

        if (capture_requested_.load()) {
            response->success = false;
            response->message = "Capture already requested";
            return;
        }

        state_ = CalibrationState::COLLECTING;
    }

    // Quick fail if no image callback has run recently. Use steady time to avoid issues when
    // header stamps are non-monotonic (e.g., multiple publishers) or system time is adjusted.
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
    const auto last_ns = last_image_receive_ns_.load(std::memory_order_relaxed);
    if (last_ns == 0 || now_ns - last_ns > static_cast<int64_t>(1e9)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_            = CalibrationState::IDLE;
        response->success = false;
        response->message = "No recent image received; ensure camera publishes /image_raw";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    if (mode_ == CalibrationMode::EXTRINSIC && !extrinsic_calibrator_) {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            state_ = CalibrationState::IDLE;
        }
        response->success = false;
        response->message = "Extrinsic calibrator not ready (waiting for CameraInfo)";
        return;
    }

    refreshCollectionParameters();

    {
        std::lock_guard<std::mutex> lock(capture_mutex_);
        capture_requested_      = true;
        capture_response_ready_ = false;
    }

    std::unique_lock<std::mutex> wait_lock(capture_mutex_);
    bool finished = capture_cv_.wait_for(
        wait_lock, std::chrono::seconds(3), [this]() { return capture_response_ready_; });

    if (!finished) {
        capture_requested_      = false;
        capture_response_ready_ = false;
        response->success       = false;
        response->message       = "Timed out waiting for a valid frame";
    } else {
        response->success       = last_capture_success_;
        response->message       = last_capture_message_;
        capture_response_ready_ = false;
    }

    wait_lock.unlock();

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (state_ != CalibrationState::CALIBRATING) {
            state_ = CalibrationState::IDLE;
        }
    }

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    }
}

void CalibrationNode::resetServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    {
        std::lock_guard<std::mutex> lock(capture_mutex_);
        capture_requested_      = false;
        capture_response_ready_ = false;
    }

    if (mode_ == CalibrationMode::INTRINSIC) {
        intrinsic_calibrator_->reset();
    } else if (extrinsic_calibrator_) {
        extrinsic_calibrator_->reset();
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_ = CalibrationState::IDLE;
        previous_corners_.clear();
        last_accepted_imu_valid_ = false;
    }

    response->success = true;
    response->message = "Calibration data reset";
    RCLCPP_INFO(this->get_logger(), "Calibration data reset");
}

void CalibrationNode::drawStatusOverlay(cv::Mat& image, double current_quality) {
    if (image.empty()) {
        return;
    }

    CalibrationState state_snapshot;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_snapshot = state_;
    }

    std::string state_text;
    cv::Scalar color;

    if (state_snapshot == CalibrationState::IDLE) {
        state_text = "Ready - Call capture_sample to store a frame";
        color      = cv::Scalar(255, 255, 0); // yellow
    } else if (state_snapshot == CalibrationState::COLLECTING) {
        state_text = "Capturing sample...";
        color      = cv::Scalar(0, 255, 0);   // green
    } else {
        state_text = "Calibrating...";
        color      = cv::Scalar(0, 165, 255); // orange
    }

    cv::putText(image, state_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.9, color, 2);

    int collected      = 0;
    double avg_quality = 0.0;
    if (mode_ == CalibrationMode::INTRINSIC) {
        collected   = intrinsic_calibrator_->getCollectedFrames();
        avg_quality = intrinsic_calibrator_->getAverageQuality();
    } else if (extrinsic_calibrator_) {
        collected   = extrinsic_calibrator_->getCollectedSamples();
        avg_quality = extrinsic_calibrator_->getAverageQuality();
    }

    double progress = std::min(
        1.0, static_cast<double>(collected) / static_cast<double>(std::max(1, target_samples_)));
    int bar_width = 300;
    int filled    = static_cast<int>(progress * bar_width);
    cv::rectangle(
        image, cv::Point(10, 50), cv::Point(10 + bar_width, 70), cv::Scalar(80, 80, 80), 1);
    cv::rectangle(
        image, cv::Point(10, 50), cv::Point(10 + filled, 70), cv::Scalar(0, 255, 0), cv::FILLED);

    std::ostringstream progress_text;
    progress_text << collected << "/" << target_samples_ << " samples  avg: " << std::fixed
                  << std::setprecision(2) << avg_quality;
    cv::putText(
        image, progress_text.str(), cv::Point(10, 95), cv::FONT_HERSHEY_SIMPLEX, 0.7,
        cv::Scalar(255, 255, 255), 2);

    if (current_quality > 0.0) {
        std::ostringstream quality_text;
        quality_text << "Current quality: " << std::fixed << std::setprecision(2)
                     << current_quality;
        cv::putText(
            image, quality_text.str(), cv::Point(10, image.rows - 20), cv::FONT_HERSHEY_SIMPLEX,
            0.7, cv::Scalar(200, 200, 0), 2);
    }
}

void CalibrationNode::triggerCalibration() {
    calibration_future_ = std::async(std::launch::async, [this]() { performCalibration(); });
}

void CalibrationNode::performCalibration() {
    bool success           = false;
    double error           = std::numeric_limits<double>::quiet_NaN();
    int collected          = 0;
    double average_quality = 0.0;

    try {
        if (mode_ == CalibrationMode::INTRINSIC) {
            success         = intrinsic_calibrator_->calibrate(camera_matrix_, dist_coeffs_, error);
            collected       = intrinsic_calibrator_->getCollectedFrames();
            average_quality = intrinsic_calibrator_->getAverageQuality();
            if (success) {
                intrinsic_calibrated_ = true;
            }
        } else {
            if (!extrinsic_calibrator_) {
                throw std::runtime_error("Extrinsic calibrator not initialized");
            }
            success         = extrinsic_calibrator_->calibrate(R_cam_to_imu_, t_cam_to_imu_, error);
            collected       = extrinsic_calibrator_->getCollectedSamples();
            average_quality = extrinsic_calibrator_->getAverageQuality();
        }
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Calibration failed with OpenCV exception: %s", e.what());
        success = false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Calibration failed with exception: %s", e.what());
        success = false;
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_ = CalibrationState::IDLE;
        previous_corners_.clear();
    }

    if (mode_ == CalibrationMode::INTRINSIC) {
        collected       = intrinsic_calibrator_->getCollectedFrames();
        average_quality = intrinsic_calibrator_->getAverageQuality();
    } else if (extrinsic_calibrator_) {
        collected       = extrinsic_calibrator_->getCollectedSamples();
        average_quality = extrinsic_calibrator_->getAverageQuality();
    }

    std::ostringstream oss;
    if (success) {
        std::string error_label = (mode_ == CalibrationMode::INTRINSIC) ? "RMS" : "Error";
        oss << (mode_ == CalibrationMode::INTRINSIC ? "Intrinsic" : "Extrinsic")
            << " calibration successful! " << error_label << ": " << std::fixed
            << std::setprecision(3) << error << " | samples: " << collected
            << " | avg quality: " << std::setprecision(2) << average_quality;
        printCalibrationResults();
    } else {
        oss << "Calibration failed. Collected " << collected << "/" << target_samples_
            << " samples.";
    }

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}

void CalibrationNode::loadQualityWeights() {
    intrinsic_weights_.corner_quality = this->get_parameter("weight_corner_quality").as_double();
    intrinsic_weights_.board_size     = this->get_parameter("weight_board_size").as_double();
    intrinsic_weights_.board_angle    = this->get_parameter("weight_board_angle").as_double();
    intrinsic_weights_.sharpness      = this->get_parameter("weight_image_sharpness").as_double();

    extrinsic_weights_.reprojection_error =
        this->get_parameter("weight_reprojection_error").as_double();
    extrinsic_weights_.imu_diversity = this->get_parameter("weight_imu_diversity").as_double();
    extrinsic_weights_.board_pose    = this->get_parameter("weight_board_pose").as_double();

    quality_scorer_.setIntrinsicWeights(intrinsic_weights_);
    quality_scorer_.setExtrinsicWeights(extrinsic_weights_);
}

void CalibrationNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    camera_info_        = *msg;
    camera_info_loaded_ = true;
    camera_info_manager_->setCameraInfo(camera_info_);

    if (mode_ == CalibrationMode::EXTRINSIC && !extrinsic_calibrator_) {
        setupExtrinsicMode();
    }
}

bool CalibrationNode::loadIntrinsicsFromCameraInfo(const sensor_msgs::msg::CameraInfo& info) {
    if (info.k.size() != 9 || info.d.size() < 5) {
        return false;
    }

    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        camera_matrix_.at<double>(i / 3, i % 3) = info.k[i];
    }

    dist_coeffs_ = cv::Mat(5, 1, CV_64F);
    for (int i = 0; i < 5; ++i) {
        dist_coeffs_.at<double>(i, 0) = info.d[i];
    }
    return true;
}

void CalibrationNode::printCalibrationResults() {
    if (mode_ == CalibrationMode::INTRINSIC) {
        std::cout << "\n=== Camera Intrinsic Calibration Results ===" << std::endl;
        std::cout << "Camera Matrix K:" << std::endl;
        std::cout << camera_matrix_ << std::endl;
        std::cout << "\nDistortion Coefficients D:" << std::endl;
        std::cout << dist_coeffs_.t() << std::endl;
        std::cout << "============================================\n" << std::endl;
    } else {
        std::cout << "\n=== Camera-IMU Extrinsic Calibration Results ===" << std::endl;
        std::cout << "Rotation Matrix R (Camera to IMU):" << std::endl;
        std::cout << R_cam_to_imu_ << std::endl;
        std::cout << "\nTranslation Vector t (Camera to IMU):" << std::endl;
        std::cout << t_cam_to_imu_.t() << std::endl;

        if (R_cam_to_imu_.rows == 3 && R_cam_to_imu_.cols == 3 && t_cam_to_imu_.rows >= 3
            && t_cam_to_imu_.cols >= 1) {
            auto printPose = [](const cv::Mat& R, const cv::Mat& t, const std::string& label) {
                tf2::Matrix3x3 tf_R(
                    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(1, 0),
                    R.at<double>(1, 1), R.at<double>(1, 2), R.at<double>(2, 0), R.at<double>(2, 1),
                    R.at<double>(2, 2));
                double roll = 0.0, pitch = 0.0, yaw = 0.0;
                tf_R.getRPY(roll, pitch, yaw);

                double tx = t.at<double>(0, 0);
                double ty = t.at<double>(1, 0);
                double tz = t.at<double>(2, 0);

                std::cout << std::fixed << std::setprecision(4);
                std::cout << "\n"
                          << label << " (RPY rad): {roll " << roll << ", pitch " << pitch
                          << ", yaw " << yaw << "}" << std::endl;
                std::cout << label << " translation (m): {x " << tx << ", y " << ty << ", z " << tz
                          << "}" << std::endl;
            };

            auto printTfSnippet =
                [this](const cv::Mat& R, const cv::Mat& t, const std::string& child_frame) {
                    tf2::Matrix3x3 tf_R(
                        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
                    tf2::Quaternion q;
                    tf_R.getRotation(q);
                    q.normalize();

                    double tx = t.at<double>(0, 0);
                    double ty = t.at<double>(1, 0);
                    double tz = t.at<double>(2, 0);

                    std::cout << std::fixed << std::setprecision(6);
                    std::cout << "\nTF (parent='" << imu_frame_ << "', child='" << child_frame
                              << "')"
                              << "  # maps child -> parent" << std::endl;
                    std::cout << "  translation (m): " << tx << " " << ty << " " << tz << std::endl;
                    std::cout << "  rotation (quat): " << q.x() << " " << q.y() << " " << q.z()
                              << " " << q.w() << std::endl;
                    std::cout << "  static TF cmd: ros2 run tf2_ros static_transform_publisher "
                              << tx << " " << ty << " " << tz << " " << q.x() << " " << q.y() << " "
                              << q.z() << " " << q.w() << " " << imu_frame_ << " " << child_frame
                              << std::endl;
                };

            // Native OpenCV camera (optical) frame: x right, y down, z forward.
            printPose(R_cam_to_imu_, t_cam_to_imu_, "Camera(optical) -> IMU");
            printTfSnippet(R_cam_to_imu_, t_cam_to_imu_, camera_optical_frame_);

            // Optional ROS camera_link-style frame: x forward, y left, z up.
            cv::Mat R_optical_to_ros =
                (cv::Mat_<double>(3, 3) << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0);
            cv::Mat R_ros_to_optical = R_optical_to_ros.t();
            cv::Mat R_camlink_to_imu = R_cam_to_imu_ * R_ros_to_optical;
            // Origins coincide between camera_link and optical in ROS, so translation stays the
            // same.
            printPose(R_camlink_to_imu, t_cam_to_imu_, "Camera(link) -> IMU");
            printTfSnippet(R_camlink_to_imu, t_cam_to_imu_, camera_link_frame_);

            // Ready-to-paste URDF/xacro args for rm_gimbal.urdf.xacro (joint: gimbal_link <-
            // camera_link).
            tf2::Matrix3x3 tf_R_camlink(
                R_camlink_to_imu.at<double>(0, 0), R_camlink_to_imu.at<double>(0, 1),
                R_camlink_to_imu.at<double>(0, 2), R_camlink_to_imu.at<double>(1, 0),
                R_camlink_to_imu.at<double>(1, 1), R_camlink_to_imu.at<double>(1, 2),
                R_camlink_to_imu.at<double>(2, 0), R_camlink_to_imu.at<double>(2, 1),
                R_camlink_to_imu.at<double>(2, 2));
            double roll = 0.0, pitch = 0.0, yaw = 0.0;
            tf_R_camlink.getRPY(roll, pitch, yaw);
            std::cout << std::fixed << std::setprecision(6);
            std::cout << "\nURDF/xacro (for joint parent='" << imu_frame_ << "', child='"
                      << camera_link_frame_ << "'):" << std::endl;
            std::cout << "  camera_xyz: \"" << t_cam_to_imu_.at<double>(0, 0) << " "
                      << t_cam_to_imu_.at<double>(1, 0) << " " << t_cam_to_imu_.at<double>(2, 0)
                      << "\"" << std::endl;
            std::cout << "  camera_rpy: \"" << roll << " " << pitch << " " << yaw << "\""
                      << std::endl;
        }

        std::cout << "================================================\n" << std::endl;
    }
}

} // namespace camera_imu_calibration

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<camera_imu_calibration::CalibrationNode>();

    // Use a multithreaded executor so the image callback can run while services
    // wait for capture responses.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
