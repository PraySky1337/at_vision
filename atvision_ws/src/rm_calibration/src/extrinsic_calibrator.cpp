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

bool ExtrinsicCalibrator::tryAddSample(
  const cv::Mat& rvec,
  const cv::Mat& tvec,
  const Eigen::Matrix3d& R_imu,
  const Eigen::Vector3d& t_imu,
  const std::vector<cv::Point2f>& corners,
  double score,
  double reprojection_error)
{
  if (score < quality_threshold_ && !best_samples_.empty()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  CalibrationSample sample;
  sample.rvec_camera = rvec.clone();
  sample.tvec_camera = tvec.clone();
  sample.R_imu = R_imu;
  sample.t_imu = t_imu;
  sample.corners = corners;
  sample.score = score;
  sample.reprojection_error = reprojection_error;

  if (best_samples_.size() < static_cast<size_t>(target_samples_)) {
    best_samples_.push(sample);
    return true;
  }

  if (!best_samples_.empty() && score > best_samples_.top().score) {
    best_samples_.pop();
    best_samples_.push(sample);
    return true;
  }

  return false;
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
    // gripper (IMU) -> base (world) as rotation matrix/translation (from TF)
    cv::Mat R_imu_base = eigenToCvMat(sample.R_imu);
    R_imu_base.convertTo(R_imu_base, CV_64F);
    if (R_imu_base.rows != 3 || R_imu_base.cols != 3) {
      throw std::runtime_error("Invalid R_imu_base size: expected 3x3");
    }
    R_gripper2base.push_back(R_imu_base.clone());             // 3x3
    cv::Mat t_imu_base = (cv::Mat_<double>(3, 1) << sample.t_imu.x(),
                           sample.t_imu.y(), sample.t_imu.z());
    t_gripper2base.push_back(t_imu_base);                     // 3x1

    // target (checkerboard) -> camera (PnP result) as rotation matrix
    cv::Mat R_target;
    cv::Rodrigues(sample.rvec_camera, R_target);
    R_target.convertTo(R_target, CV_64F);
    if (R_target.rows != 3 || R_target.cols != 3) {
      throw std::runtime_error("Invalid R_target size: expected 3x3");
    }
    R_target2cam.push_back(R_target);                         // 3x3

    cv::Mat tvec_cam = sample.tvec_camera.clone();
    tvec_cam.convertTo(tvec_cam, CV_64F);
    if (tvec_cam.rows == 1 && tvec_cam.cols == 3) {
      tvec_cam = tvec_cam.t();
    }
    if (tvec_cam.rows != 3 || tvec_cam.cols != 1) {
      throw std::runtime_error("Invalid tvec size: expected 3x1");
    }
    t_target2cam.push_back(tvec_cam);
  }

  cv::Mat R_cam2imu, tvec_cam2imu;
  cv::calibrateHandEye(
    R_gripper2base, t_gripper2base,
    R_target2cam, t_target2cam,
    R_cam2imu, tvec_cam2imu,
    cv::CALIB_HAND_EYE_TSAI);

  R_cam2imu.convertTo(R_cam_to_imu, CV_64F);
  tvec_cam2imu.convertTo(t_cam_to_imu, CV_64F);
  if (R_cam_to_imu.rows != 3 || R_cam_to_imu.cols != 3) {
    throw std::runtime_error("Hand-eye rotation output has unexpected size");
  }
  if (t_cam_to_imu.rows == 1 && t_cam_to_imu.cols == 3) {
    t_cam_to_imu = t_cam_to_imu.t();
  }
  if (t_cam_to_imu.rows != 3 || t_cam_to_imu.cols != 1) {
    throw std::runtime_error("Hand-eye translation output has unexpected size");
  }

  // Consistency metric based on AX=XB residuals between consecutive poses
  auto computeInvMul = [](const cv::Mat& R1, const cv::Mat& t1,
                          const cv::Mat& R2, const cv::Mat& t2,
                          cv::Mat& R_rel, cv::Mat& t_rel) {
    // inv(T2) * T1
    R_rel = R2.t() * R1;
    t_rel = R2.t() * (t1 - t2);
  };
  auto computeMulInv = [](const cv::Mat& R1, const cv::Mat& t1,
                          const cv::Mat& R2, const cv::Mat& t2,
                          cv::Mat& R_rel, cv::Mat& t_rel) {
    // T2 * inv(T1)
    R_rel = R2 * R1.t();
    t_rel = t2 - R_rel * t1;
  };

  double rotation_error_sum = 0.0;
  double translation_error_sum = 0.0;
  int pair_count = 0;

  for (size_t i = 1; i < samples.size(); ++i) {
    cv::Mat R_A, t_A, R_B, t_B;
    computeInvMul(
      R_gripper2base[i - 1], t_gripper2base[i - 1],
      R_gripper2base[i], t_gripper2base[i],
      R_A, t_A);
    computeMulInv(
      R_target2cam[i - 1], t_target2cam[i - 1],
      R_target2cam[i], t_target2cam[i],
      R_B, t_B);

    cv::Mat rot_res = R_A * R_cam_to_imu - R_cam_to_imu * R_B;
    rotation_error_sum += cv::norm(rot_res, cv::NORM_L2);

    cv::Mat lhs_t = t_A + R_A * t_cam_to_imu;
    cv::Mat rhs_t = R_cam_to_imu * t_B + t_cam_to_imu;
    translation_error_sum += cv::norm(lhs_t - rhs_t);
    ++pair_count;
  }

  error = (pair_count > 0)
            ? (rotation_error_sum + translation_error_sum) /
                static_cast<double>(pair_count)
            : 0.0;

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
