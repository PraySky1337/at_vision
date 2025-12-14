#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace camera_imu_calibration
{

struct IntrinsicWeights
{
  double corner_quality{0.3};
  double board_size{0.2};
  double board_angle{0.3};
  double sharpness{0.2};
};

struct ExtrinsicWeights
{
  double reprojection_error{0.4};
  double imu_diversity{0.3};
  double board_pose{0.3};
};

class QualityScorer
{
public:
  QualityScorer() = default;

  void setIntrinsicWeights(const IntrinsicWeights& weights) { intrinsic_weights_ = weights; }
  void setExtrinsicWeights(const ExtrinsicWeights& weights) { extrinsic_weights_ = weights; }

  double scoreIntrinsicFrame(
    const cv::Mat& image,
    const std::vector<cv::Point2f>& corners,
    const cv::Size& board_size,
    const std::vector<cv::Point2f>& previous_corners) const;

  double scoreExtrinsicSample(
    const cv::Mat& image,
    const std::vector<cv::Point2f>& corners,
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    const Eigen::Matrix3d& R_imu,
    double reprojection_error) const;

private:
  double calculateCornerQuality(
    const cv::Mat& gray,
    const std::vector<cv::Point2f>& corners) const;

  double calculateBoardSizeScore(
    const std::vector<cv::Point2f>& corners,
    const cv::Size& image_size) const;

  double calculateAngleScore(
    const std::vector<cv::Point2f>& corners,
    const cv::Size& board_size) const;

  double calculateSharpness(const cv::Mat& gray) const;

  double calculateDiversity(
    const std::vector<cv::Point2f>& corners,
    const std::vector<cv::Point2f>& previous,
    const cv::Size& image_size) const;

  double calculateBoardPoseScore(const cv::Mat& rvec) const;
  double calculateReprojectionScore(double reprojection_error) const;
  double calculateImuDiversity(const Eigen::Matrix3d& R_imu) const;

  IntrinsicWeights intrinsic_weights_;
  ExtrinsicWeights extrinsic_weights_;
};

}  // namespace camera_imu_calibration
