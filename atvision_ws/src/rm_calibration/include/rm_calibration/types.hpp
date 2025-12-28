#pragma once

#include <Eigen/Dense>

#include <opencv2/core.hpp>

#include <cstddef>

namespace rm_calibration
{

struct Pose
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

struct IntrinsicCalibrationResult
{
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  double reprojection_error_px = 0.0;
  std::size_t used_samples = 0;
};

struct ExtrinsicCalibrationResult
{
  cv::Mat R_camera2gimbal;
  cv::Mat t_camera2gimbal;
  Eigen::Vector3d misalignment_ypr_deg = Eigen::Vector3d::Zero();
  std::size_t used_samples = 0;

  // URDF-compatible output (gimbal_link -> camera_link in ROS frame)
  Eigen::Vector3d urdf_xyz = Eigen::Vector3d::Zero();   // translation [m]
  Eigen::Vector3d urdf_rpy = Eigen::Vector3d::Zero();   // roll, pitch, yaw [rad]
  Eigen::Vector3d assembly_error_deg = Eigen::Vector3d::Zero(); // human-readable error
};

}  // namespace rm_calibration

