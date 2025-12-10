#pragma once

#include "armor_detector_ov/armor_types.hpp"

#include <array>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace rm_auto_aim {

class PnPSolver {
public:
    PnPSolver() = default;
    PnPSolver(const std::array<double, 9>& camera_matrix,
              const std::vector<double>& dist_coeffs)
    {
        init(camera_matrix, dist_coeffs);
    }

    // 初始化：相机内参和畸变（支持5参数：k1,k2,p1,p2,k3）
    bool init(const std::array<double, 9>& camera_matrix,
              const std::vector<double>& dist_coeffs);

    // 基础 PnP：OpenCV + IPPE，输出相机坐标系下装甲板位姿（rvec/tvec）
    bool solvePnP(const ArmorObject& armor, cv::Mat& rvec, cv::Mat& tvec);

    // 基于 Ceres 的 yaw-only BA（FYT 风格）：
    //
    // 1. 内部先调用 solvePnP 获取 rvec/tvec 作为初值；
    // 2. 在 IMU 坐标系下只优化 yaw（pitch 由装甲类型约束：outpost 与其他不同）；
    // 3. 平移 tvec 不变
    //
    // 参数：
    //   armor      : 当前装甲板检测结果（提供 2D 点和 cls）
    //   R_imu_cam  : imu->camera 的旋转（从 TF 求出）
    // 输出：
    //   rvec, tvec : 优化后的相机系位姿（tvec 沿用 solvePnP，rvec 为 BA 后的 R）
    bool solvePnPWithBA(const ArmorObject& armor,
                        const Eigen::Matrix3d& R_imu_cam,
                        cv::Mat& rvec,
                        cv::Mat& tvec);

    // 工具：像平中心距离（用于优先级或排序）
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
