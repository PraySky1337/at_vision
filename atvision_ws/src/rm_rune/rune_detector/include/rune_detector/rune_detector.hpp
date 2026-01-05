
#ifndef RUNE_DETECTOR_RUNE_DETECTOR_HPP_
#define RUNE_DETECTOR_RUNE_DETECTOR_HPP_

// std
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <tuple>
#include <vector>
// third party
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/rune_target.hpp"
// #include "rm_utils/math/pnp_solver.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
// project
#include "rune_detector/types.hpp"

namespace fyt::rune {

class RuneDetector {
public:
    RuneDetector();
    RuneDetector(
        EnemyColor detect_color, int arrow_threshold, int target_threshold, int rcenter_threshold);

    bool detect(const cv::Mat& frame, int image_width, int image_height);

    Eigen::Matrix4d solve(
        const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const rclcpp::Time& stamp,
        tf2_ros::Buffer& tf2_buffer);

    void setKeyPoints();
    void setLocalRoi();
    void setGlobalRoi();

    cv::Mat localMask;
    cv::Rect2f globalRoi;
    std::vector<cv::Rect2f> targetROIs;
    cv::Rect2f centerRoi;
    cv::Mat arrowImg;
    cv::Mat targetImg;
    cv::Mat rCenterImg;
    CenterR rcenter;
    std::vector<Target> targets;
    std::vector<Arrow> arrows;
    Status status;
    rclcpp::Time timestamp;

private:
    cv::Mat processPictures(const cv::Mat& src);
     bool isArrowRoiMatched(const Arrow& arrow, const cv::Rect2f& roi, const cv::Rect2f& globalRoi);

    bool detectCenterR();
    bool detectAllArrows();
    bool detectAllTargets();

    EnemyColor detect_color_;
    int arrow_threshold_;
    int target_threshold_;
    int rcenter_threshold_;

    int image_width_;
    int image_height_;

    // 上一帧追踪的靶位置（用于目标连续性）
    cv::Point2f last_tracked_target_center_{0, 0};
    double last_timestamp_=0;
    bool has_last_target_{false};
};
} // namespace fyt::rune
#endif // RUNE_DETECTOR_RUNE_DETECTOR_HPP_
