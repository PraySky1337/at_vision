#pragma once

#pragma once

#include <Eigen/Dense>
#include <functional>

namespace at {

// ---- 通用卡尔曼滤波器基类模板 ----
// N_X: 状态维度
// N_Z: 观测维度
template <int N_X, int N_Z>
class KalmanFilterBase {
public:
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>;
    using MatrixZX = Eigen::Matrix<double, N_Z, N_X>;
    using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>;
    using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
    using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;
    using UpdateQFunc = std::function<MatrixXX()>;
    using UpdateRFunc = std::function<MatrixZZ(const MatrixZ1&)>;

    virtual ~KalmanFilterBase() = default;

    virtual void setState(const MatrixX1& x0) noexcept = 0;
    virtual MatrixX1 predict() noexcept = 0;
    virtual MatrixX1 update(const MatrixZ1& z) noexcept = 0;

    virtual const MatrixX1& state() const noexcept = 0;
    virtual const MatrixXX& covariance() const noexcept = 0;

    virtual void setUpdateQFunc(const UpdateQFunc& f) noexcept { update_Q_ = f; }
    virtual void setUpdateRFunc(const UpdateRFunc& f) noexcept { update_R_ = f; }

    // ---- 模型函数（非虚模板成员，可在派生类中隐藏重载）----
    template <typename F>
    void setPredictFunc(const F& f) noexcept {
        (void)f;
        // 默认空实现；派生类中可隐藏重载
    }

    template <typename H>
    void setMeasureFunc(const H& h) noexcept {
        (void)h;
        // 默认空实现；派生类中可隐藏重载
    }

protected:
    UpdateQFunc update_Q_;
    UpdateRFunc update_R_;
};

}  // namespace fyt
