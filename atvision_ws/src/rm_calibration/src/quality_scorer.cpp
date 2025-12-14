#include "rm_calibration/quality_scorer.hpp"

#include <algorithm>
#include <cmath>

namespace camera_imu_calibration
{

namespace
{
inline double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;
constexpr double kQuarterPi = kPi / 4.0;
}  // namespace

double QualityScorer::scoreIntrinsicFrame(
  const cv::Mat& image,
  const std::vector<cv::Point2f>& corners,
  const cv::Size& board_size,
  const std::vector<cv::Point2f>& previous_corners) const
{
  if (corners.empty() || image.empty()) {
    return 0.0;
  }

  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }

  double corner_quality = calculateCornerQuality(gray, corners);
  double board_size_score = calculateBoardSizeScore(corners, image.size());
  double angle_score = calculateAngleScore(corners, board_size);
  double sharpness_score = calculateSharpness(gray);
  double diversity_score = calculateDiversity(corners, previous_corners, image.size());

  double angle_diversity = 0.6 * angle_score + 0.4 * diversity_score;

  double weight_sum = intrinsic_weights_.corner_quality +
                      intrinsic_weights_.board_size +
                      intrinsic_weights_.board_angle +
                      intrinsic_weights_.sharpness;
  if (weight_sum <= 0.0) {
    return 0.0;
  }

  double score = intrinsic_weights_.corner_quality * corner_quality +
                 intrinsic_weights_.board_size * board_size_score +
                 intrinsic_weights_.board_angle * angle_diversity +
                 intrinsic_weights_.sharpness * sharpness_score;
  return clamp01(score / weight_sum);
}

double QualityScorer::scoreExtrinsicSample(
  const cv::Mat& image,
  const std::vector<cv::Point2f>& corners,
  const cv::Mat& rvec,
  const cv::Mat& tvec,
  const Eigen::Matrix3d& R_imu,
  double reprojection_error) const
{
  (void)tvec;
  if (corners.empty() || image.empty() || rvec.empty()) {
    return 0.0;
  }

  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }

  double corner_quality = calculateCornerQuality(gray, corners);
  double sharpness_score = calculateSharpness(gray);
  double reprojection_score = calculateReprojectionScore(reprojection_error);
  double imu_diversity = calculateImuDiversity(R_imu);
  double board_pose_score = calculateBoardPoseScore(rvec);

  constexpr double corner_bonus = 0.1;
  constexpr double sharpness_bonus = 0.1;

  double weight_sum = extrinsic_weights_.reprojection_error +
                      extrinsic_weights_.imu_diversity +
                      extrinsic_weights_.board_pose +
                      corner_bonus +
                      sharpness_bonus;
  if (weight_sum <= 0.0) {
    return 0.0;
  }

  double score = extrinsic_weights_.reprojection_error * reprojection_score +
                 extrinsic_weights_.imu_diversity * imu_diversity +
                 extrinsic_weights_.board_pose * board_pose_score +
                 corner_bonus * corner_quality +
                 sharpness_bonus * sharpness_score;
  return clamp01(score / weight_sum);
}

double QualityScorer::calculateCornerQuality(
  const cv::Mat& gray,
  const std::vector<cv::Point2f>& corners) const
{
  if (corners.empty() || gray.empty()) {
    return 0.0;
  }

  cv::Mat eigen;
  cv::cornerMinEigenVal(gray, eigen, 3, 3);

  double min_val = 0.0;
  double max_val = 0.0;
  cv::minMaxLoc(eigen, &min_val, &max_val);
  if (max_val <= 1e-6) {
    return 0.0;
  }

  double score_sum = 0.0;
  for (const auto& c : corners) {
    int x = std::max(0, std::min(eigen.cols - 1, static_cast<int>(std::round(c.x))));
    int y = std::max(0, std::min(eigen.rows - 1, static_cast<int>(std::round(c.y))));
    score_sum += eigen.at<float>(y, x) / max_val;
  }

  return clamp01(score_sum / static_cast<double>(corners.size()));
}

double QualityScorer::calculateBoardSizeScore(
  const std::vector<cv::Point2f>& corners,
  const cv::Size& image_size) const
{
  if (corners.empty() || image_size.area() == 0) {
    return 0.0;
  }

  cv::Rect bounding = cv::boundingRect(corners);
  double area_ratio = static_cast<double>(bounding.area()) /
                      static_cast<double>(image_size.area());

  double optimal = 0.2;   // ideal area ratio
  double spread = 0.15;   // how far from optimal we start to penalize
  double score = std::exp(-std::pow((area_ratio - optimal) / spread, 2.0));
  return clamp01(score);
}

double QualityScorer::calculateAngleScore(
  const std::vector<cv::Point2f>& corners,
  const cv::Size& board_size) const
{
  if (corners.size() < 2 || board_size.area() == 0) {
    return 0.0;
  }

  int last_col_index = board_size.width - 1;
  int last_row_index = (board_size.height - 1) * board_size.width;
  if (last_col_index >= static_cast<int>(corners.size()) ||
      last_row_index >= static_cast<int>(corners.size())) {
    return 0.0;
  }

  cv::Point2f vx = corners[last_col_index] - corners[0];
  double angle = std::abs(std::atan2(vx.y, vx.x));

  double preferred = kQuarterPi;  // around 45 degrees
  double spread = kPi / 6.0;
  double score = std::exp(-std::pow((angle - preferred) / spread, 2.0));
  return clamp01(score);
}

double QualityScorer::calculateSharpness(const cv::Mat& gray) const
{
  if (gray.empty()) {
    return 0.0;
  }

  cv::Mat lap;
  cv::Laplacian(gray, lap, CV_64F);
  cv::Scalar mean, stddev;
  cv::meanStdDev(lap, mean, stddev);

  double variance = stddev[0] * stddev[0];
  double score = std::tanh(variance / 5000.0);
  return clamp01(score);
}

double QualityScorer::calculateDiversity(
  const std::vector<cv::Point2f>& corners,
  const std::vector<cv::Point2f>& previous,
  const cv::Size& image_size) const
{
  if (corners.empty() || previous.empty() || image_size.area() == 0 ||
      corners.size() != previous.size()) {
    return 0.0;
  }

  double distance_sum = 0.0;
  for (size_t i = 0; i < corners.size(); ++i) {
    distance_sum += cv::norm(corners[i] - previous[i]);
  }

  double avg_distance = distance_sum / static_cast<double>(corners.size());
  double diagonal = std::hypot(image_size.width, image_size.height);
  if (diagonal <= 0.0) {
    return 0.0;
  }

  double normalized = avg_distance / (0.25 * diagonal);
  return clamp01(normalized);
}

double QualityScorer::calculateBoardPoseScore(const cv::Mat& rvec) const
{
  if (rvec.empty()) {
    return 0.0;
  }

  cv::Mat R;
  cv::Rodrigues(rvec, R);
  cv::Vec3d normal(
    R.at<double>(0, 2),
    R.at<double>(1, 2),
    R.at<double>(2, 2));

  double facing = std::abs(normal[2]);
  return clamp01(1.0 - facing);  // prefer some tilt away from the camera
}

double QualityScorer::calculateReprojectionScore(double reprojection_error) const
{
  double normalized_error = std::max(0.0, reprojection_error);
  double score = 1.0 / (1.0 + normalized_error);
  return clamp01(score);
}

double QualityScorer::calculateImuDiversity(const Eigen::Matrix3d& R_imu) const
{
  double trace = R_imu.trace();
  double cos_angle = (trace - 1.0) / 2.0;
  cos_angle = std::min(1.0, std::max(-1.0, cos_angle));
  double angle = std::acos(cos_angle);

  double score = angle / kHalfPi;  // 0 rad -> 0, 90deg -> 1
  return clamp01(score);
}

}  // namespace camera_imu_calibration
