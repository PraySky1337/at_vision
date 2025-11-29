// Copyright 2022 Chen Jun
#pragma once
#include "armor_detector_ov/armor_types.hpp"
#include <array>
#include <opencv2/core.hpp>
#include <vector>

namespace rm_auto_aim {

class PnPSolver {
public:
    PnPSolver() = default;
    PnPSolver(const std::array<double, 9>& camera_matrix, const std::vector<double>& dist_coeffs) {
        init(camera_matrix, dist_coeffs);
    }

    // 初始化：相机内参和畸变（支持5参数：k1,k2,p1,p2,k3）
    bool init(const std::array<double, 9>& camera_matrix, const std::vector<double>& dist_coeffs);

    // 无先验的 PnP（OpenCV, IPPE）
    bool solvePnP(const ArmorObject& armor, cv::Mat& rvec, cv::Mat& tvec);

    // 带**姿态软约束**（Ceres 精修）——先验以“四元数（xyzw）”给出
    // sigma_rad：每轴角度标准差（弧度），常用等方：{deg2rad(5),deg2rad(5),deg2rad(5)}
    bool solvePnPWithPrior(
        const ArmorObject& armor, const cv::Vec4d& q_prior_xyzw, const cv::Vec3d& sigma_rad,
        cv::Mat& rvec, cv::Mat& tvec);

    // 工具：像平中心距离
    float calculateDistanceToCenter(const cv::Point2f& image_point);

private:
    void buildModelPoints();

private:
    cv::Mat camera_matrix_; // 3x3, CV_64F
    cv::Mat dist_coeffs_;   // 1x5, CV_64F (k1,k2,p1,p2,k3)
    std::vector<cv::Point3f> small_armor_points_;
    std::vector<cv::Point3f> large_armor_points_;
};

} // namespace rm_auto_aim
