// Copyright
#include "armor_detector_ov/pnp_solver.hpp"

#include <array>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/calib3d.hpp>
#include <vector>

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

bool PnPSolver::init(
    const std::array<double, 9>& camera_matrix, const std::vector<double>& dist_coeffs) {
    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_matrix.data())).clone();
    if (dist_coeffs.size() >= 5) {
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double*>(dist_coeffs.data())).clone();
    } else {
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
        for (size_t i = 0; i < dist_coeffs.size(); ++i)
            dist_coeffs_.at<double>(0, static_cast<int>(i)) = dist_coeffs[i];
    }
    buildModelPoints();
    return !camera_matrix_.empty();
}
void PnPSolver::buildModelPoints() {
    small_armor_points_.clear();
    large_armor_points_.clear();

    constexpr double small_half_y = SMALL_ARMOR_WIDTH * 0.5 / 1000.0;
    constexpr double small_half_z = SMALL_ARMOR_HEIGHT * 0.5 / 1000.0;
    constexpr double large_half_y = LARGE_ARMOR_WIDTH * 0.5 / 1000.0;
    constexpr double large_half_z = LARGE_ARMOR_HEIGHT * 0.5 / 1000.0;

    // 模型坐标：x forward, y left, z up；从左上开始逆时针
    small_armor_points_.emplace_back(0, small_half_y, small_half_z);
    small_armor_points_.emplace_back(0, small_half_y, -small_half_z);
    small_armor_points_.emplace_back(0, -small_half_y, -small_half_z);
    small_armor_points_.emplace_back(0, -small_half_y, small_half_z);

    large_armor_points_.emplace_back(0, large_half_y, large_half_z);
    large_armor_points_.emplace_back(0, large_half_y, -large_half_z);
    large_armor_points_.emplace_back(0, -large_half_y, -large_half_z);
    large_armor_points_.emplace_back(0, -large_half_y, large_half_z);
}

bool PnPSolver::solvePnP(const ArmorObject& armor, cv::Mat& rvec, cv::Mat& tvec) {
    vector<cv::Point2f> img_pts = {armor.apex[0], armor.apex[1], armor.apex[2], armor.apex[3]};
    if (armor_cls_to_string(armor.cls) == "1") {
        return cv::solvePnP(
            large_armor_points_, img_pts, camera_matrix_, dist_coeffs_, rvec, tvec,
            /*useGuess=*/false, cv::SOLVEPNP_IPPE);
    } else {
        return cv::solvePnP(
            small_armor_points_, img_pts, camera_matrix_, dist_coeffs_, rvec, tvec,
            /*useGuess=*/false, cv::SOLVEPNP_IPPE);
    }
}

// -------------------- Ceres 残差 --------------------

struct ReprojWithDistCost {
    ReprojWithDistCost(
        const cv::Point2f& z, const cv::Point3f& Xw, const double* K, const double* D)
        : u_(z.x)
        , v_(z.y)
        , X_{Xw.x, Xw.y, Xw.z}
        , fx_(K[0])
        , fy_(K[4])
        , cx_(K[2])
        , cy_(K[5])
        , k1_(D[0])
        , k2_(D[1])
        , p1_(D[2])
        , p2_(D[3])
        , k3_(D[4]) {}

    template <typename T>
    bool operator()(const T* const rvec, const T* const tvec, T* res) const {
        T Xc[3];
        T XwT[3] = {T(X_[0]), T(X_[1]), T(X_[2])};
        ceres::AngleAxisRotatePoint(rvec, XwT, Xc);
        Xc[0] += tvec[0];
        Xc[1] += tvec[1];
        Xc[2] += tvec[2];

        T x = Xc[0] / Xc[2];
        T y = Xc[1] / Xc[2];

        T r2     = x * x + y * y;
        T radial = T(1.0) + T(k1_) * r2 + T(k2_) * r2 * r2 + T(k3_) * r2 * r2 * r2;
        T x_tan  = T(2.0) * T(p1_) * x * y + T(p2_) * (r2 + T(2.0) * x * x);
        T y_tan  = T(p1_) * (r2 + T(2.0) * y * y) + T(2.0) * T(p2_) * x * y;

        T x_d = x * radial + x_tan;
        T y_d = y * radial + y_tan;

        T uhat = T(fx_) * x_d + T(cx_);
        T vhat = T(fy_) * y_d + T(cy_);

        res[0] = uhat - T(u_);
        res[1] = vhat - T(v_);
        return true;
    }

    double u_, v_;
    double X_[3];
    double fx_, fy_, cx_, cy_;
    double k1_, k2_, p1_, p2_, k3_;
};

// SO(3) 对数先验：r = Log( Q_prior^{-1} * Q_current )
struct SO3LogPriorCost {
    explicit SO3LogPriorCost(const cv::Vec4d& q_prior_xyzw, const cv::Vec3d& sigma_rad) {
        // 归一化并转为 wxyz
        const double nx = q_prior_xyzw[0], ny = q_prior_xyzw[1], nz = q_prior_xyzw[2],
                     nw = q_prior_xyzw[3];
        const double n  = std::sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
        qp_[0]          = (n > 0) ? (nw / n) : 1.0; // w
        qp_[1]          = (n > 0) ? (nx / n) : 0.0; // x
        qp_[2]          = (n > 0) ? (ny / n) : 0.0; // y
        qp_[3]          = (n > 0) ? (nz / n) : 0.0; // z

        iw_[0] = 1.0 / std::max(1e-9, sigma_rad[0]);
        iw_[1] = 1.0 / std::max(1e-9, sigma_rad[1]);
        iw_[2] = 1.0 / std::max(1e-9, sigma_rad[2]);
    }

    template <typename T>
    bool operator()(const T* const rvec, T* res) const {
        // 当前角轴 -> 四元数（wxyz）
        T qc[4];
        ceres::AngleAxisToQuaternion(rvec, qc);

        // q_rel = q_prior^{-1} * q_current
        const T qw = T(qp_[0]), qx = T(qp_[1]), qy = T(qp_[2]), qz = T(qp_[3]);
        // prior^{-1} = [w,-x,-y,-z]
        T qpi[4] = {qw, -qx, -qy, -qz};
        // 乘法（wxyz）
        T qr[4];
        qr[0] = qpi[0] * qc[0] - qpi[1] * qc[1] - qpi[2] * qc[2] - qpi[3] * qc[3];
        qr[1] = qpi[0] * qc[1] + qpi[1] * qc[0] + qpi[2] * qc[3] - qpi[3] * qc[2];
        qr[2] = qpi[0] * qc[2] - qpi[1] * qc[3] + qpi[2] * qc[0] + qpi[3] * qc[1];
        qr[3] = qpi[0] * qc[3] + qpi[1] * qc[2] - qpi[2] * qc[1] + qpi[3] * qc[0];

        // Log(q_rel) → angle-axis
        T aa[3];
        ceres::QuaternionToAngleAxis(qr, aa);

        res[0] = T(iw_[0]) * aa[0];
        res[1] = T(iw_[1]) * aa[1];
        res[2] = T(iw_[2]) * aa[2];
        return true;
    }

    double qp_[4]; // prior (wxyz)
    double iw_[3]; // 1/sigma
};

// -------------------- 有先验精修 --------------------

bool PnPSolver::solvePnPWithPrior(
    const ArmorObject& armor, const cv::Vec4d& q_prior_xyzw, const cv::Vec3d& sigma_rad,
    cv::Mat& rvec, cv::Mat& tvec) {
    // 1) 先用 IPPE 求初值
    if (!solvePnP(armor, rvec, tvec))
        return false;

    // 2) Ceres 构图
    ceres::Problem problem;

    const double K[9] = {camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(0, 1),
                         camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 0),
                         camera_matrix_.at<double>(1, 1), camera_matrix_.at<double>(1, 2),
                         camera_matrix_.at<double>(2, 0), camera_matrix_.at<double>(2, 1),
                         camera_matrix_.at<double>(2, 2)};
    const double D[5] = {
        dist_coeffs_.at<double>(0, 0), dist_coeffs_.at<double>(0, 1), dist_coeffs_.at<double>(0, 2),
        dist_coeffs_.at<double>(0, 3), dist_coeffs_.at<double>(0, 4)};

    std::vector<cv::Point2f> img_pts{armor.apex[0], armor.apex[1], armor.apex[2], armor.apex[3]};
    const auto& obj_pts = isLargeArmor(armor) ? large_armor_points_ : small_armor_points_;

    // 参数块
    double r[3] = {rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)};
    double t[3] = {tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};

    for (int i = 0; i < 4; ++i) {
        auto* cost = new ceres::AutoDiffCostFunction<ReprojWithDistCost, 2, 3, 3>(
            new ReprojWithDistCost(img_pts[i], obj_pts[i], K, D));
        problem.AddResidualBlock(cost, new ceres::HuberLoss(1.0), r, t);
    }

    // SO(3) 对数先验（四元数）
    {
        auto* prior = new ceres::AutoDiffCostFunction<SO3LogPriorCost, 3, 3>(
            new SO3LogPriorCost(q_prior_xyzw, sigma_rad));
        problem.AddResidualBlock(prior, nullptr, r);
    }

    // 可选：平移弱正则（如需）
    // struct Tikhonov3 { template<typename T> bool operator()(const T* const x, T* rr) const {
    //   rr[0]=T(1e-3)*x[0]; rr[1]=T(1e-3)*x[1]; rr[2]=T(1e-3)*x[2]; return true; } };
    // problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Tikhonov3,3,3>(new Tikhonov3),
    // nullptr, t);

    ceres::Solver::Options opts;
    opts.linear_solver_type           = ceres::DENSE_QR;
    opts.max_num_iterations           = 50;
    opts.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);

    rvec = (cv::Mat_<double>(3, 1) << r[0], r[1], r[2]);
    tvec = (cv::Mat_<double>(3, 1) << t[0], t[1], t[2]);
    return summary.IsSolutionUsable();
}

// -------------------- 工具 --------------------

float PnPSolver::calculateDistanceToCenter(const cv::Point2f& image_point) {
    float cx = static_cast<float>(camera_matrix_.at<double>(0, 2));
    float cy = static_cast<float>(camera_matrix_.at<double>(1, 2));
    return static_cast<float>(cv::norm(image_point - cv::Point2f(cx, cy)));
}

} // namespace rm_auto_aim
