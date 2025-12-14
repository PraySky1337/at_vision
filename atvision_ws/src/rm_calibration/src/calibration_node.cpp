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

    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_, camera_info_url_);
    camera_info_       = camera_info_manager_->getCameraInfo();
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

    // Subscribe to image and camera info topics
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&CalibrationNode::imageCallback, this, std::placeholders::_1));
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, 10, std::bind(&CalibrationNode::cameraInfoCallback, this, std::placeholders::_1));

    // Create services
    calibrate_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/calibrate", std::bind(
                           &CalibrationNode::calibrateServiceCallback, this, std::placeholders::_1,
                           std::placeholders::_2));

    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reset", std::bind(
                       &CalibrationNode::resetServiceCallback, this, std::placeholders::_1,
                       std::placeholders::_2));

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
    this->declare_parameter("calibration_duration", 30.0);
    this->declare_parameter("quality_threshold", 0.2);
    this->declare_parameter("display_fps", 10.0);
    this->declare_parameter("min_sample_interval", 0.2);
    this->declare_parameter("imu_frame", "gimbal_link");
    this->declare_parameter("base_frame", "odom");
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
    calibration_duration_  = this->get_parameter("calibration_duration").as_double();
    display_fps_           = this->get_parameter("display_fps").as_double();
    min_sample_interval_   = this->get_parameter("min_sample_interval").as_double();
    imu_frame_             = this->get_parameter("imu_frame").as_string();
    base_frame_            = this->get_parameter("base_frame").as_string();
    camera_name_           = this->get_parameter("camera_name").as_string();
    camera_info_url_       = this->get_parameter("camera_info_url").as_string();
    camera_info_topic_     = this->get_parameter("camera_info_topic").as_string();
    enable_preview_window_ = this->get_parameter("enable_preview_window").as_bool();

    // Backward compatibility with required_frames
    int legacy_required = this->get_parameter("required_frames").as_int();
    if (target_samples_ <= 0) {
        target_samples_ = legacy_required;
    }

    target_samples_       = std::max(1, target_samples_);
    quality_threshold_    = std::max(0.0, std::min(1.0, quality_threshold_));
    calibration_duration_ = std::max(0.1, calibration_duration_);
    min_sample_interval_  = std::max(0.0, min_sample_interval_);

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
        RCLCPP_ERROR(this->get_logger(), "Invalid CameraInfo; cannot initialize extrinsic calibrator");
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

void CalibrationNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
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

    CalibrationState state_snapshot;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_snapshot = state_;
    }

    if (mode_ == CalibrationMode::INTRINSIC) {
        std::vector<cv::Point2f> corners;
        bool found = intrinsic_calibrator_->detectCorners(cv_ptr->image, corners, display_image);

        if (found) {
            current_quality = quality_scorer_.scoreIntrinsicFrame(
                cv_ptr->image, corners, cv::Size(board_width_, board_height_), previous_corners_);
            previous_corners_ = corners;

            if (state_snapshot == CalibrationState::COLLECTING) {
                rclcpp::Time now = this->now();
                if (last_sample_time_.nanoseconds() == 0 ||
                    (now - last_sample_time_).seconds() >= min_sample_interval_) {
                    intrinsic_calibrator_->tryAddSample(
                        corners, cv_ptr->image.size(), current_quality);
                    last_sample_time_ = now;
                }
                if (intrinsic_calibrator_->getCollectedFrames() >= target_samples_) {
                    triggerCalibration();
                }
            }
        }
    } else {
        if (!extrinsic_calibrator_) {
            display_image = cv_ptr->image.clone();
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for camera info to initialize extrinsic calibrator...");
            drawStatusOverlay(display_image, current_quality);

            if (preview_pub_) {
                std_msgs::msg::Header header = msg->header;
                auto preview_msg =
                    cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, display_image)
                        .toImageMsg();
                preview_pub_.publish(preview_msg);
            }
            return;
        }

        Eigen::Matrix3d R_imu;
        if (getIMURotation(R_imu)) {
            std::vector<cv::Point2f> corners;
            cv::Mat rvec, tvec;
            double reprojection_error = 0.0;
            bool found                = extrinsic_calibrator_->detectSample(
                cv_ptr->image, R_imu, corners, display_image, rvec, tvec, reprojection_error);

            if (found) {
                current_quality = quality_scorer_.scoreExtrinsicSample(
                    cv_ptr->image, corners, rvec, tvec, R_imu, reprojection_error);
                previous_corners_ = corners;

                if (state_snapshot == CalibrationState::COLLECTING) {
                    rclcpp::Time now = this->now();
                    if (last_sample_time_.nanoseconds() == 0 ||
                        (now - last_sample_time_).seconds() >= min_sample_interval_) {
                        extrinsic_calibrator_->tryAddSample(
                            rvec, tvec, R_imu, corners, current_quality, reprojection_error);
                        last_sample_time_ = now;
                    }
                }
            }
        } else {
            display_image = cv_ptr->image.clone();
        }
    }

    drawStatusOverlay(display_image, current_quality);

    bool show_image = true;
    if (display_fps_ > 0.0) {
        rclcpp::Time now = this->now();
        double dt        = (last_display_time_.nanoseconds() == 0) ? (1.0 / display_fps_)
                                                                   : (now - last_display_time_).seconds();
        if (dt >= (1.0 / display_fps_)) {
            last_display_time_ = now;
        } else {
            show_image = false;
        }
    }

    if (show_image && enable_preview_window_) {
        cv::imshow("Calibration", display_image);
        cv::waitKey(1);
    }

    if (preview_pub_) {
        std_msgs::msg::Header header = msg->header;
        auto preview_msg =
            cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, display_image)
                .toImageMsg();
        preview_pub_.publish(preview_msg);
    }
}

bool CalibrationNode::getIMURotation(Eigen::Matrix3d& R_imu) {
    try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);

        tf2::Quaternion q(
            transform.transform.rotation.x, transform.transform.rotation.y,
            transform.transform.rotation.z, transform.transform.rotation.w);

        tf2::Matrix3x3 m(q);
        R_imu << m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2];

        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000, "Could not get transform: %s", ex.what());
        return false;
    }
}

void CalibrationNode::calibrateServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (state_ != CalibrationState::IDLE) {
            response->success = false;
            response->message = "Calibration already in progress";
            return;
        }
    }

    // Refresh collection parameters in case they were updated dynamically
    target_samples_       = this->get_parameter("target_samples").as_int();
    quality_threshold_    = this->get_parameter("quality_threshold").as_double();
    calibration_duration_ = this->get_parameter("calibration_duration").as_double();
    display_fps_          = this->get_parameter("display_fps").as_double();
    loadQualityWeights();

    target_samples_       = std::max(1, target_samples_);
    quality_threshold_    = std::max(0.0, std::min(1.0, quality_threshold_));
    calibration_duration_ = std::max(0.1, calibration_duration_);

    if (mode_ == CalibrationMode::INTRINSIC) {
        intrinsic_calibrator_->reset();
        intrinsic_calibrator_->configureCollection(target_samples_, quality_threshold_);
    } else {
        if (!extrinsic_calibrator_) {
            response->success = false;
            response->message = "Extrinsic calibrator not ready (waiting for CameraInfo)";
            return;
        }
        extrinsic_calibrator_->reset();
        extrinsic_calibrator_->configureCollection(target_samples_, quality_threshold_);
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        previous_corners_.clear();
        state_                 = CalibrationState::COLLECTING;
        collection_start_time_ = this->now();
        last_sample_time_      = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    }

    if (calibration_timer_) {
        calibration_timer_->cancel();
        calibration_timer_.reset();
    }

    calibration_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(calibration_duration_),
        std::bind(&CalibrationNode::triggerCalibration, this));

    std::ostringstream oss;
    oss << "Started collecting calibration data for " << calibration_duration_ << "s, target "
        << target_samples_ << " samples (threshold " << quality_threshold_ << ")";

    response->success = true;
    response->message = oss.str();

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void CalibrationNode::resetServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    if (calibration_timer_) {
        calibration_timer_->cancel();
        calibration_timer_.reset();
    }

  if (mode_ == CalibrationMode::INTRINSIC) {
    intrinsic_calibrator_->reset();
  } else {
    if (extrinsic_calibrator_) {
      extrinsic_calibrator_->reset();
    }
  }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_ = CalibrationState::IDLE;
        previous_corners_.clear();
        if (calibration_timer_) {
            calibration_timer_->cancel();
            calibration_timer_.reset();
        }
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
    rclcpp::Time start_time;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_snapshot = state_;
        start_time     = collection_start_time_;
    }

    std::string state_text;
    cv::Scalar color;

    if (state_snapshot == CalibrationState::IDLE) {
        state_text = "Ready - Call calibrate service to start";
        color      = cv::Scalar(255, 255, 0);
    } else if (state_snapshot == CalibrationState::COLLECTING) {
        double elapsed   = (this->now() - start_time).seconds();
        double remaining = std::max(0.0, calibration_duration_ - elapsed);
        state_text       = "Collecting... " + std::to_string(static_cast<int>(remaining)) + "s";
        color            = cv::Scalar(0, 255, 0);
    } else {
        state_text = "Calibrating...";
        color      = cv::Scalar(0, 165, 255);
    }

    cv::putText(image, state_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.9, color, 2);

  int collected = 0;
  double avg_quality = 0.0;
  if (mode_ == CalibrationMode::INTRINSIC) {
    collected = intrinsic_calibrator_->getCollectedFrames();
    avg_quality = intrinsic_calibrator_->getAverageQuality();
  } else if (extrinsic_calibrator_) {
    collected = extrinsic_calibrator_->getCollectedSamples();
    avg_quality = extrinsic_calibrator_->getAverageQuality();
  }

    if (state_snapshot == CalibrationState::COLLECTING) {
        double progress = std::min(
            1.0,
            static_cast<double>(collected) / static_cast<double>(std::max(1, target_samples_)));
        int bar_width = 300;
        int filled    = static_cast<int>(progress * bar_width);
        cv::rectangle(
            image, cv::Point(10, 50), cv::Point(10 + bar_width, 70), cv::Scalar(80, 80, 80), 1);
        cv::rectangle(
            image, cv::Point(10, 50), cv::Point(10 + filled, 70), cv::Scalar(0, 255, 0),
            cv::FILLED);

        std::ostringstream progress_text;
        progress_text << collected << "/" << target_samples_ << " samples  avg: " << std::fixed
                      << std::setprecision(2) << avg_quality;
        cv::putText(
            image, progress_text.str(), cv::Point(10, 95), cv::FONT_HERSHEY_SIMPLEX, 0.7,
            cv::Scalar(255, 255, 255), 2);
    } else {
        std::ostringstream info;
        info << "Samples: " << collected << " avg quality: " << std::fixed << std::setprecision(2)
             << avg_quality;
        cv::putText(
            image, info.str(), cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7,
            cv::Scalar(255, 255, 255), 2);
    }

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
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (state_ != CalibrationState::COLLECTING) {
            return;
        }
        state_ = CalibrationState::CALIBRATING;
    }

    if (calibration_timer_) {
        calibration_timer_->cancel();
        calibration_timer_.reset();
    }

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
        std::cout << "================================================\n" << std::endl;
    }
}

} // namespace camera_imu_calibration

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<camera_imu_calibration::CalibrationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
