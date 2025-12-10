// pnp_solver.cpp
// Copyright
#include "pnp_solver.hpp"

#include <array>
#include <cmath>
#include <vector>

#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// ==== 装甲板尺寸 ：mm ====
#ifndef SMALL_ARMOR_WIDTH
# define SMALL_ARMOR_WIDTH 135.0
#endif
#ifndef SMALL_ARMOR_HEIGHT
# define SMALL_ARMOR_HEIGHT 55.0
#endif
#ifndef LARGE_ARMOR_WIDTH
# define LARGE_ARMOR_WIDTH 230.0
#endif
#ifndef LARGE_ARMOR_HEIGHT
# define LARGE_ARMOR_HEIGHT 55.0
#endif

namespace rm_auto_aim {

using std::vector;

//===================== Ceres yaw-only 残差（FYT 风格） =====================

// 注意：
// - 只优化 IMU 坐标系下的 yaw
// - pitch 由外部 R_pitch 给定
// - 平移 t_cam_armor 不参与优化（保持 PnP 的结果）
struct YawOnlyReprojError {
    YawOnlyReprojError(const Eigen::Vector3d& Pw,
                       const Eigen::Vector2d& uv,
                       const Eigen::Matrix3d& R_cam_imu,
                       const Eigen::Matrix3d& R_pitch,
                       const Eigen::Vector3d& t_cam_armor,
                       const Eigen::Matrix3d& K)
        : Pw_(Pw)
        , uv_(uv)
        , R_cam_imu_(R_cam_imu)
        , R_pitch_(R_pitch)
        , t_(t_cam_armor)
        , K_(K)
    {}

    template <typename T>
    bool operator()(const T* const yaw, T* residuals) const
    {
        // R_yaw (IMU 坐标系下绕 Z 轴旋转)
        const T cy = ceres::cos(yaw[0]);
        const T sy = ceres::sin(yaw[0]);
        Eigen::Matrix<T,3,3> R_yaw;
        R_yaw <<  cy, -sy, T(0),
                  sy,  cy, T(0),
                  T(0), T(0), T(1);

        // R = R_cam_imu * R_yaw * R_pitch
        Eigen::Matrix<T,3,3> R_cam_imu = R_cam_imu_.template cast<T>();
        Eigen::Matrix<T,3,3> R_pitch   = R_pitch_.template cast<T>();
        Eigen::Matrix<T,3,3> R         = R_cam_imu * R_yaw * R_pitch;

        // 3D 模型点（装甲板坐标系）
        Eigen::Matrix<T,3,1> Pw;
        Pw << T(Pw_.x()), T(Pw_.y()), T(Pw_.z());

        // 平移（装甲 -> 相机），不优化
        Eigen::Matrix<T,3,1> t;
        t << T(t_.x()), T(t_.y()), T(t_.z());

        // 相机坐标系
        Eigen::Matrix<T,3,1> Pc = R * Pw + t;
        const T& Xc = Pc(0);
        const T& Yc = Pc(1);
        const T& Zc = Pc(2);

        if (Zc <= T(0)) {
            residuals[0] = T(0);
            residuals[1] = T(0);
            return true;
        }

        // 归一化像平面坐标
        Eigen::Matrix<T,3,1> p_norm;
        p_norm << Xc / Zc, Yc / Zc, T(1);

        // 像素坐标（不再使用畸变，和 FYT 的 BA 实现一致）
        Eigen::Matrix<T,3,3> K = K_.template cast<T>();
        Eigen::Matrix<T,3,1> p_img = K * p_norm;

        const T u = p_img(0);
        const T v = p_img(1);

        residuals[0] = u - T(uv_.x());
        residuals[1] = v - T(uv_.y());
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& Pw,
                                       const Eigen::Vector2d& uv,
                                       const Eigen::Matrix3d& R_cam_imu,
                                       const Eigen::Matrix3d& R_pitch,
                                       const Eigen::Vector3d& t_cam_armor,
                                       const Eigen::Matrix3d& K)
    {
        return new ceres::AutoDiffCostFunction<YawOnlyReprojError, 2, 1>(
            new YawOnlyReprojError(Pw, uv, R_cam_imu, R_pitch, t_cam_armor, K));
    }

    Eigen::Vector3d Pw_;
    Eigen::Vector2d uv_;
    Eigen::Matrix3d R_cam_imu_;
    Eigen::Matrix3d R_pitch_;
    Eigen::Vector3d t_;
    Eigen::Matrix3d K_;
};

//===================== PnPSolver 实现 =====================

bool PnPSolver::init(const std::array<double, 9>& camera_matrix,
                     const std::vector<double>& dist_coeffs)
{
    camera_matrix_ = cv::Mat(3, 3, CV_64F,
                             const_cast<double*>(camera_matrix.data())).clone();

    if (dist_coeffs.size() >= 5) {
        dist_coeffs_ = cv::Mat(1, 5, CV_64F,
                               const_cast<double*>(dist_coeffs.data())).clone();
    } else {
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
        for (size_t i = 0; i < dist_coeffs.size(); ++i) {
            dist_coeffs_.at<double>(0, static_cast<int>(i)) = dist_coeffs[i];
        }
    }

    buildModelPoints();
    return !camera_matrix_.empty();
}

void PnPSolver::buildModelPoints()
{
    small_armor_points_.clear();
    large_armor_points_.clear();

    constexpr double small_half_y = SMALL_ARMOR_WIDTH  * 0.5 / 1000.0;
    constexpr double small_half_z = SMALL_ARMOR_HEIGHT * 0.5 / 1000.0;
    constexpr double large_half_y = LARGE_ARMOR_WIDTH  * 0.5 / 1000.0;
    constexpr double large_half_z = LARGE_ARMOR_HEIGHT * 0.5 / 1000.0;

    // 模型坐标：x forward, y left, z up；从左上开始逆时针
    // small
    small_armor_points_.emplace_back(0,  small_half_y,  small_half_z);
    small_armor_points_.emplace_back(0,  small_half_y, -small_half_z);
    small_armor_points_.emplace_back(0, -small_half_y, -small_half_z);
    small_armor_points_.emplace_back(0, -small_half_y,  small_half_z);

    // large
    large_armor_points_.emplace_back(0,  large_half_y,  large_half_z);
    large_armor_points_.emplace_back(0,  large_half_y, -large_half_z);
    large_armor_points_.emplace_back(0, -large_half_y, -large_half_z);
    large_armor_points_.emplace_back(0, -large_half_y,  large_half_z);
}

bool PnPSolver::solvePnP(const ArmorObject& armor, cv::Mat& rvec, cv::Mat& tvec)
{
    vector<cv::Point2f> img_pts = {
        armor.apex[0], armor.apex[1],
        armor.apex[2], armor.apex[3]
    };

    const std::string cls_str = armor_cls_to_string(armor.cls);

    if (cls_str == "1") {
        // large armor
        return cv::solvePnP(
            large_armor_points_,
            img_pts,
            camera_matrix_,
            dist_coeffs_,
            rvec,
            tvec,
            /*useExtrinsicGuess=*/false,
            cv::SOLVEPNP_IPPE);
    } else {
        // small armor
        return cv::solvePnP(
            small_armor_points_,
            img_pts,
            camera_matrix_,
            dist_coeffs_,
            rvec,
            tvec,
            /*useExtrinsicGuess=*/false,
            cv::SOLVEPNP_IPPE);
    }
}

bool PnPSolver::solvePnPWithBA(const ArmorObject& armor,
                               const Eigen::Matrix3d& R_imu_cam,
                               cv::Mat& rvec,
                               cv::Mat& tvec)
{
    // 1. 先用 OpenCV IPPE 求一个 6DoF 初值
    if (!solvePnP(armor, rvec, tvec)) {
        return false;
    }

    // 2. rvec / tvec -> Eigen 形式
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);

    Eigen::Matrix3d R_cam_armor;
    cv::cv2eigen(R_cv, R_cam_armor);

    Eigen::Vector3d t_cam_armor(
        tvec.at<double>(0),
        tvec.at<double>(1),
        tvec.at<double>(2));

    // imu->camera
    Eigen::Matrix3d R_imu_camera = R_imu_cam;
    // camera->imu
    Eigen::Matrix3d R_cam_imu = R_imu_camera.transpose();

    // imu->armor
    Eigen::Matrix3d R_imu_armor = R_imu_camera * R_cam_armor;

    // 2.1 计算初始 yaw
    double theta_by_sin = std::asin(-R_imu_armor(0, 1));
    double theta_by_cos = std::acos( R_imu_armor(1, 1));
    double yaw_init     = 0.0;

    if (std::abs(theta_by_sin) > 1e-5) {
        yaw_init = (theta_by_sin > 0.0) ? theta_by_cos : -theta_by_cos;
    } else {
        yaw_init = (R_imu_armor(1, 1) > 0.0) ? 0.0 : M_PI;
    }

    // 2.2 装甲板 pitch 约束（outpost / 非 outpost）
    const std::string cls_str = armor_cls_to_string(armor.cls);
    constexpr double FIFTEEN_DEG_RAD = 15.0 * M_PI / 180.0;

    double armor_pitch_rad = FIFTEEN_DEG_RAD;
    if (cls_str == "outpost") {
        armor_pitch_rad = -FIFTEEN_DEG_RAD;
    }

    Eigen::Matrix3d R_pitch;
    {
        double cp = std::cos(armor_pitch_rad);
        double sp = std::sin(armor_pitch_rad);
        R_pitch <<  cp, 0.0, sp,
                    0.0,1.0, 0.0,
                   -sp, 0.0, cp;
    }

    // 2.3 相机内参矩阵 K（不带畸变）
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0,0) = camera_matrix_.at<double>(0,0);
    K(1,1) = camera_matrix_.at<double>(1,1);
    K(0,2) = camera_matrix_.at<double>(0,2);
    K(1,2) = camera_matrix_.at<double>(1,2);

    // 2.4 3D 模型点（small / large 与 solvePnP 一致）
    std::vector<Eigen::Vector3d> obj_pts;
    {
        const std::vector<cv::Point3f>& src =
            (cls_str == "1") ? large_armor_points_ : small_armor_points_;
        obj_pts.reserve(src.size());
        for (const auto& p : src) {
            obj_pts.emplace_back(p.x, p.y, p.z);
        }
    }

    // 2.5 2D 像素观测
    std::vector<Eigen::Vector2d> img_pts;
    img_pts.emplace_back(armor.apex[0].x, armor.apex[0].y);
    img_pts.emplace_back(armor.apex[1].x, armor.apex[1].y);
    img_pts.emplace_back(armor.apex[2].x, armor.apex[2].y);
    img_pts.emplace_back(armor.apex[3].x, armor.apex[3].y);

    // 3. 构建 Ceres 问题：只优化 yaw
    double yaw = yaw_init;

    ceres::Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

    for (size_t i = 0; i < obj_pts.size(); ++i) {
        ceres::CostFunction* cost = YawOnlyReprojError::Create(
            obj_pts[i], img_pts[i], R_cam_imu, R_pitch, t_cam_armor, K);

        problem.AddResidualBlock(cost, loss_function, &yaw);
    }

    // yaw 可选范围约束
    problem.SetParameterLowerBound(&yaw, 0, -M_PI);
    problem.SetParameterUpperBound(&yaw, 0,  M_PI);

    ceres::Solver::Options options;
    options.linear_solver_type           = ceres::DENSE_QR;
    options.max_num_iterations           = 20;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!std::isfinite(yaw)) {
        // 优化失败就退回 solvePnP 的结果
        return true;
    }

    // 4. 用优化后的 yaw 重建 R_cam_armor_refined = R_cam_imu * R_yaw * R_pitch
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    Eigen::Matrix3d R_yaw;
    R_yaw <<  cy, -sy, 0.0,
              sy,  cy, 0.0,
              0.0, 0.0, 1.0;

    Eigen::Matrix3d R_cam_armor_refined = R_cam_imu * R_yaw * R_pitch;

    // 写回 rvec，tvec 保持不变
    cv::Mat R_refined_cv;
    cv::eigen2cv(R_cam_armor_refined, R_refined_cv);
    cv::Rodrigues(R_refined_cv, rvec);

    return true;
}

// -------------------- 工具 --------------------

float PnPSolver::calculateDistanceToCenter(const cv::Point2f& image_point)
{
    float cx = static_cast<float>(camera_matrix_.at<double>(0, 2));
    float cy = static_cast<float>(camera_matrix_.at<double>(1, 2));
    return static_cast<float>(cv::norm(image_point - cv::Point2f(cx, cy)));
}

} // namespace rm_auto_aim
