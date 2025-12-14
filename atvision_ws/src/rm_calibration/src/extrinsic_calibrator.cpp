#include "rm_calibration/extrinsic_calibrator.hpp"

#include <algorithm>
#include <iostream>

namespace camera_imu_calibration
{

ExtrinsicCalibrator::ExtrinsicCalibrator(
  int board_width,
  int board_height,
  double square_size,
  const cv::Mat& camera_matrix,
  const cv::Mat& dist_coeffs,
  int target_samples)
: board_width_(board_width),
  board_height_(board_height),
  square_size_(square_size),
  target_samples_(target_samples),
  quality_threshold_(0.0),
  camera_matrix_(camera_matrix.clone()),
  dist_coeffs_(dist_coeffs.clone())
{
}

bool ExtrinsicCalibrator::detectSample(
  const cv::Mat& image,
  const Eigen::Matrix3d& R_imu,
  std::vector<cv::Point2f>& corners,
  cv::Mat& display_image,
  cv::Mat& rvec,
  cv::Mat& tvec,
  double& reprojection_error) const
{
  (void)R_imu;
  image.copyTo(display_image);
  cv::Size board_size(board_width_, board_height_);
  rvec = cv::Mat();
  tvec = cv::Mat();
  reprojection_error = 0.0;
  
  bool found = cv::findChessboardCorners(
    image,
    board_size,
    corners,
    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
  
  if (found) {
    // Refine corners
    cv::Mat gray;
    if (image.channels() == 3) {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = image;
    }
    
    cv::cornerSubPix(
      gray,
      corners,
      cv::Size(11, 11),
      cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    
    // Solve PnP to get camera pose relative to checkerboard
    std::vector<cv::Point3f> object_points = createObjectPoints();
    bool success = cv::solvePnP(
      object_points,
      corners,
      camera_matrix_,
      dist_coeffs_,
      rvec,
      tvec,
      false,
      cv::SOLVEPNP_ITERATIVE);
    
    if (success) {
      reprojection_error = computeReprojectionError(object_points, corners, rvec, tvec);

      // Draw
      cv::drawChessboardCorners(display_image, board_size, corners, found);
      return true;
    }
  }

  return false;
}

void ExtrinsicCalibrator::configureCollection(int target_samples, double quality_threshold)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  target_samples_ = target_samples;
  quality_threshold_ = quality_threshold;
}

void ExtrinsicCalibrator::tryAddSample(
  const cv::Mat& rvec,
  const cv::Mat& tvec,
  const Eigen::Matrix3d& R_imu,
  const std::vector<cv::Point2f>& corners,
  double score,
  double reprojection_error)
{
  if (score < quality_threshold_ && !best_samples_.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  CalibrationSample sample;
  sample.rvec_camera = rvec.clone();
  sample.tvec_camera = tvec.clone();
  sample.R_imu = R_imu;
  sample.corners = corners;
  sample.score = score;
  sample.reprojection_error = reprojection_error;

  if (best_samples_.size() < static_cast<size_t>(target_samples_)) {
    best_samples_.push(sample);
  } else if (!best_samples_.empty() && score > best_samples_.top().score) {
    best_samples_.pop();
    best_samples_.push(sample);
  }
}

bool ExtrinsicCalibrator::calibrate(
  cv::Mat& R_cam_to_imu,
  cv::Mat& t_cam_to_imu,
  double& error)
{
  const auto samples = getSamplesSnapshot();
  if (samples.size() < 3) {
    std::cerr << "Not enough samples. Need at least 3, have "
              << samples.size() << std::endl;
    return false;
  }

  std::vector<cv::Mat> R_gripper2base, t_gripper2base;
  std::vector<cv::Mat> R_target2cam, t_target2cam;
  R_gripper2base.reserve(samples.size());
  t_gripper2base.reserve(samples.size());
  R_target2cam.reserve(samples.size());
  t_target2cam.reserve(samples.size());

  for (const auto& sample : samples) {
    // gripper (IMU) -> base (world)
    cv::Mat R_base_imu = eigenToCvMat(sample.R_imu);
    cv::Mat R_imu_base = R_base_imu.t();
    cv::Mat rvec_imu_base;
    cv::Rodrigues(R_imu_base, rvec_imu_base);
    R_gripper2base.push_back(rvec_imu_base);
    t_gripper2base.push_back(cv::Mat::zeros(3, 1, CV_64F));

    // target (checkerboard) -> camera (PnP result)
    R_target2cam.push_back(sample.rvec_camera.clone());
    t_target2cam.push_back(sample.tvec_camera.clone());
  }

  cv::Mat rvec_cam2imu, tvec_cam2imu;
  cv::calibrateHandEye(
    R_gripper2base, t_gripper2base,
    R_target2cam, t_target2cam,
    rvec_cam2imu, tvec_cam2imu,
    cv::CALIB_HAND_EYE_TSAI);

  cv::Rodrigues(rvec_cam2imu, R_cam_to_imu);
  t_cam_to_imu = tvec_cam2imu.clone();

  // Simple consistency metric: average rotation difference to samples
  error = 0.0;
  for (const auto& sample : samples) {
    cv::Mat R_cam;
    cv::Rodrigues(sample.rvec_camera, R_cam);
    cv::Mat R_est = R_cam_to_imu * eigenToCvMat(sample.R_imu);
    cv::Mat diff = R_est - R_cam;
    error += cv::norm(diff);
  }
  error /= samples.size();

  return true;
}

std::vector<cv::Point3f> ExtrinsicCalibrator::createObjectPoints() const
{
  std::vector<cv::Point3f> corners;
  for (int i = 0; i < board_height_; i++) {
    for (int j = 0; j < board_width_; j++) {
      corners.push_back(cv::Point3f(j * square_size_, i * square_size_, 0.0f));
    }
  }
  return corners;
}

Eigen::Matrix3d ExtrinsicCalibrator::cvMatToEigen(const cv::Mat& mat) const
{
  Eigen::Matrix3d eigen_mat;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      eigen_mat(i, j) = mat.at<double>(i, j);
    }
  }
  return eigen_mat;
}

cv::Mat ExtrinsicCalibrator::eigenToCvMat(const Eigen::Matrix3d& mat) const
{
  cv::Mat cv_mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cv_mat.at<double>(i, j) = mat(i, j);
    }
  }
  return cv_mat;
}

double ExtrinsicCalibrator::computeReprojectionError(
  const std::vector<cv::Point3f>& object_points,
  const std::vector<cv::Point2f>& image_points,
  const cv::Mat& rvec,
  const cv::Mat& tvec) const
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected);

  double sum_error = 0.0;
  for (size_t i = 0; i < projected.size(); ++i) {
    sum_error += cv::norm(projected[i] - image_points[i]);
  }
  return projected.empty() ? 0.0 : sum_error / static_cast<double>(projected.size());
}

ExtrinsicCalibrator::SampleContainer ExtrinsicCalibrator::getSamplesSnapshot() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  auto queue_copy = best_samples_;
  SampleContainer samples;
  while (!queue_copy.empty()) {
    samples.push_back(queue_copy.top());
    queue_copy.pop();
  }

  std::sort(samples.begin(), samples.end(),
            [](const CalibrationSample& a, const CalibrationSample& b) { return a.score > b.score; });
  return samples;
}

int ExtrinsicCalibrator::getCollectedSamples() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return static_cast<int>(best_samples_.size());
}

double ExtrinsicCalibrator::getAverageQuality() const
{
  const auto samples = getSamplesSnapshot();
  if (samples.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (const auto& s : samples) {
    sum += s.score;
  }
  return sum / static_cast<double>(samples.size());
}

void ExtrinsicCalibrator::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  while (!best_samples_.empty()) {
    best_samples_.pop();
  }
}

} // namespace camera_imu_calibration
