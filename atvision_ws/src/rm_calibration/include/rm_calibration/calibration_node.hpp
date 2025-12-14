#ifndef CAMERA_IMU_CALIBRATION__CALIBRATION_NODE_HPP_
#define CAMERA_IMU_CALIBRATION__CALIBRATION_NODE_HPP_

#include <atomic>
#include <camera_info_manager/camera_info_manager.hpp>
#include <condition_variable>
#include <cstdint>
#include <cv_bridge/cv_bridge.h>
#include <future>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rm_calibration/extrinsic_calibrator.hpp"
#include "rm_calibration/intrinsic_calibrator.hpp"
#include "rm_calibration/quality_scorer.hpp"

namespace camera_imu_calibration {

enum class CalibrationMode { INTRINSIC, EXTRINSIC };

class CalibrationNode : public rclcpp::Node {
public:
    CalibrationNode();
    ~CalibrationNode() = default;

    enum class CalibrationState { IDLE, COLLECTING, CALIBRATING };

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void declareParameters();
    void loadParameters();
    void setupIntrinsicMode();
    void setupExtrinsicMode();
    void refreshCollectionParameters();
    void drawStatusOverlay(cv::Mat& image, double current_quality);
    void triggerCalibration();
    void performCalibration();
    void loadQualityWeights();
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    bool loadIntrinsicsFromCameraInfo(const sensor_msgs::msg::CameraInfo& info);
    void handleCaptureRequestIntrinsic(
        bool found, const std::vector<cv::Point2f>& corners, const cv::Size& image_size,
        double quality);
    void handleCaptureRequestExtrinsic(
        bool imu_ready, bool found, const std::vector<cv::Point2f>& corners, const cv::Mat& rvec,
        const cv::Mat& tvec, const Eigen::Matrix3d& R_imu, const Eigen::Vector3d& t_imu,
        double quality, double reprojection_error);

    // Service callbacks
    void calibrateServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void captureSampleServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void resetServiceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Helper methods
    bool getIMUPose(const rclcpp::Time& stamp, Eigen::Matrix3d& R_imu, Eigen::Vector3d& t_imu);
    void printCalibrationResults();

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_sample_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    image_transport::Publisher preview_pub_;
    rclcpp::CallbackGroup::SharedPtr image_callback_group_;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    CalibrationMode mode_;
    int board_width_;
    int board_height_;
    double square_size_;
    int target_samples_;
    double quality_threshold_;
    double display_fps_;
    double max_tf_time_diff_;
    std::string imu_frame_;
    std::string base_frame_;
    std::string camera_optical_frame_;
    std::string camera_link_frame_;
    std::string camera_name_;
    std::string camera_info_url_;
    std::string camera_info_topic_;
    bool enable_preview_window_{false};
    IntrinsicWeights intrinsic_weights_;
    ExtrinsicWeights extrinsic_weights_;

    // Calibrators
    std::unique_ptr<IntrinsicCalibrator> intrinsic_calibrator_;
    std::unique_ptr<ExtrinsicCalibrator> extrinsic_calibrator_;
    QualityScorer quality_scorer_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_;
    bool camera_info_loaded_{false};
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Calibration results
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat R_cam_to_imu_;
    cv::Mat t_cam_to_imu_;
    bool intrinsic_calibrated_;
    Eigen::Matrix3d last_accepted_imu_R_{Eigen::Matrix3d::Identity()};
    bool last_accepted_imu_valid_{false};

    CalibrationState state_{CalibrationState::IDLE};
    std::future<void> calibration_future_;
    std::vector<cv::Point2f> previous_corners_;
    rclcpp::Time last_display_time_;
    std::atomic_bool capture_requested_{false};
    bool capture_response_ready_{false};
    bool last_capture_success_{false};
    std::string last_capture_message_;
    rclcpp::Time last_image_stamp_;
    std::atomic<int64_t> last_image_receive_ns_{0};
    std::condition_variable capture_cv_;
    std::mutex capture_mutex_;
    std::mutex data_mutex_;
};

} // namespace camera_imu_calibration

#endif // CAMERA_IMU_CALIBRATION__CALIBRATION_NODE_HPP_
