#pragma once

// 无迹卡尔曼滤波器
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <functional>

#include "kalman_filter_base.hpp"

namespace at {
template <int N_X, int N_Z, class PredictFunc, class MeasureFunc>
class UnscentedKalmanFilter : public KalmanFilterBase<N_X, N_Z> {
public:
    using Base = KalmanFilterBase<N_X, N_Z>;
    using typename Base::MatrixXX;
    using typename Base::MatrixXZ;
    using typename Base::MatrixZX;
    using typename Base::MatrixZZ;
    using typename Base::MatrixX1;
    using typename Base::MatrixZ1;
    using typename Base::UpdateQFunc;
    using typename Base::UpdateRFunc;

    explicit UnscentedKalmanFilter(
        const PredictFunc& f,
        const MeasureFunc& h,
        const UpdateQFunc& u_q,
        const UpdateRFunc& u_r,
        const MatrixXX& P0,
        double alpha = 1e-2,
        double beta  = 2.0,
        double kappa = 0.0)
        : f_(f), h_(h), alpha_(alpha), beta_(beta), kappa_(kappa) {
        this->update_Q_ = u_q;
        this->update_R_ = u_r;
        P_post_ = P0;
        initWeights();
    }

    void setState(const MatrixX1& x0) noexcept override { x_post_ = x0; }

    MatrixX1 predict() noexcept override {
        const double c = lambda_ + N_X;
        Eigen::Matrix<double, N_X, 2 * N_X + 1> X;
        sigmaPoints(x_post_, P_post_, c, X);

        for (int i = 0; i < 2 * N_X + 1; ++i) Xf_.col(i) = f_(X.col(i));

        x_pri_.setZero();
        for (int i = 0; i < 2 * N_X + 1; ++i) x_pri_ += Wm_(i) * Xf_.col(i);

        P_pri_.setZero();
        for (int i = 0; i < 2 * N_X + 1; ++i) {
            auto dx = Xf_.col(i) - x_pri_;
            P_pri_ += Wc_(i) * (dx * dx.transpose());
        }
        Q_ = this->update_Q_();
        P_pri_ += Q_;
        x_post_ = x_pri_;
        P_post_ = P_pri_;
        return x_pri_;
    }

    MatrixX1 update(const MatrixZ1& z) noexcept override {
        Eigen::Matrix<double, N_Z, 2 * N_X + 1> Zsig;
        for (int i = 0; i < 2 * N_X + 1; ++i) Zsig.col(i) = h_(Xf_.col(i));

        MatrixZ1 z_pri = MatrixZ1::Zero();
        for (int i = 0; i < 2 * N_X + 1; ++i) z_pri += Wm_(i) * Zsig.col(i);

        MatrixZZ S = MatrixZZ::Zero();
        MatrixXZ Cxz = MatrixXZ::Zero();

        for (int i = 0; i < 2 * N_X + 1; ++i) {
            MatrixZ1 dz = Zsig.col(i) - z_pri;
            MatrixX1 dx = Xf_.col(i) - x_pri_;
            S += Wc_(i) * (dz * dz.transpose());
            Cxz += Wc_(i) * (dx * dz.transpose());
        }
        R_ = this->update_R_(z);
        S += R_;

        K_ = Cxz * S.inverse();
        x_post_ = x_pri_ + K_ * (z - z_pri);
        P_post_ = P_pri_ - K_ * S * K_.transpose();
        return x_post_;
    }

    void setPredictFunc(const PredictFunc& f) noexcept { f_ = f; }
    void setMeasureFunc(const MeasureFunc& h) noexcept { h_ = h; }

    const MatrixX1& state() const noexcept override { return x_post_; }
    const MatrixXX& covariance() const noexcept override { return P_post_; }

private:
    // sigma 点生成
    void sigmaPoints(const MatrixX1& x, const MatrixXX& P, double c,
                     Eigen::Matrix<double, N_X, 2 * N_X + 1>& X) const {
        Eigen::LLT<MatrixXX> llt(P * c);
        MatrixXX S = llt.matrixL();
        X.col(0) = x;
        for (int i = 0; i < N_X; ++i) {
            X.col(i + 1)       = x + S.col(i);
            X.col(i + 1 + N_X) = x - S.col(i);
        }
    }

    void initWeights() {
        lambda_ = alpha_ * alpha_ * (N_X + kappa_) - N_X;
        Wm_.setConstant(1.0 / (2.0 * (N_X + lambda_)));
        Wc_ = Wm_;
        Wm_(0) = lambda_ / (N_X + lambda_);
        Wc_(0) = lambda_ / (N_X + lambda_) + (1.0 - alpha_ * alpha_ + beta_);
    }

private:
    PredictFunc f_;
    MeasureFunc h_;

    double alpha_, beta_, kappa_;
    double lambda_{0.0};

    Eigen::Matrix<double, 2 * N_X + 1, 1> Wm_;
    Eigen::Matrix<double, 2 * N_X + 1, 1> Wc_;

    MatrixXX Q_{MatrixXX::Zero()};
    MatrixZZ R_{MatrixZZ::Zero()};
    MatrixXZ K_{MatrixXZ::Zero()};

    MatrixXX P_pri_{MatrixXX::Identity()};
    MatrixXX P_post_{MatrixXX::Identity()};
    MatrixX1 x_pri_{MatrixX1::Zero()};
    MatrixX1 x_post_{MatrixX1::Zero()};

    Eigen::Matrix<double, N_X, 2 * N_X + 1> Xf_{Eigen::Matrix<double, N_X, 2 * N_X + 1>::Zero()};
};

} // namespace at
