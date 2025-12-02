#pragma once
#include <Eigen/Dense>
#include <cmath>

#include "kalman_filter_base.hpp"

namespace at {

template <int N_X, int N_Z, class PredictFunc, class MeasureFunc>
class UnscentedKalmanFilter : public KalmanFilterBase<N_X, N_Z> {
public:
    using Base        = KalmanFilterBase<N_X, N_Z>;
    using MatrixXX    = typename Base::MatrixXX;
    using MatrixXZ    = typename Base::MatrixXZ;
    using MatrixZX    = typename Base::MatrixZX;
    using MatrixZZ    = typename Base::MatrixZZ;
    using MatrixX1    = typename Base::MatrixX1;
    using MatrixZ1    = typename Base::MatrixZ1;
    using UpdateQFunc = typename Base::UpdateQFunc;
    using UpdateRFunc = typename Base::UpdateRFunc;

    static constexpr int N_SIGMA = 2 * N_X + 1;

    explicit UnscentedKalmanFilter(
        const PredictFunc& f, const MeasureFunc& h, const UpdateQFunc& u_q, const UpdateRFunc& u_r,
        const MatrixXX& P0, double alpha = 1e-2, double beta = 2.0, double kappa = 0.0,
        double gating_threshold = -1.0) // <0 关闭马氏门限
        : f_(f)
        , h_(h)
        , alpha_(alpha)
        , beta_(beta)
        , kappa_(kappa)
        , gating_threshold_(gating_threshold) {
        this->update_Q_ = u_q;
        this->update_R_ = u_r;
        P_post_         = P0;
        initWeights();
        Xf_.setZero();
        Zsig_.setZero();
    }

    void setState(const MatrixX1& x0) noexcept override { x_post_ = x0; }

    // 预测：数值稳定 + 复用缓存 + 协方差对称化 + 正定性保护
    MatrixX1 predict() noexcept override {
        const double c = lambda_ + N_X;
        sigmaPoints(x_post_, P_post_, c, Xsig_); // 生成 sigma 点到 Xsig_
        // 传播
        for (int i = 0; i < N_SIGMA; ++i)
            f_(Xsig_.col(i).data(), Xf_.col(i).data());

        // 均值
        x_pri_.setZero();
        for (int i = 0; i < N_SIGMA; ++i)
            x_pri_.noalias() += Wm_(i) * Xf_.col(i);

        // 协方差
        P_pri_.setZero();
        for (int i = 0; i < N_SIGMA; ++i) {
            MatrixX1 dx = Xf_.col(i) - x_pri_;
            P_pri_.noalias() += Wc_(i) * (dx * dx.transpose());
        }

        Q_ = this->update_Q_();
        P_pri_.noalias() += Q_;

        // 对称化保证数值稳定
        P_pri_ = (P_pri_ + P_pri_.transpose()) * 0.5;

        x_post_ = x_pri_;
        P_post_ = P_pri_;
        return x_pri_;
    }

    // 更新：LDLT 代替逆、马氏距离门限（可选）、对称化
    MatrixX1 update(const MatrixZ1& z) noexcept override {
        // 量测预测
        for (int i = 0; i < N_SIGMA; ++i)
            h_(Xf_.col(i).data(), Zsig_.col(i).data());

        MatrixZ1 z_pri = MatrixZ1::Zero();
        for (int i = 0; i < N_SIGMA; ++i)
            z_pri.noalias() += Wm_(i) * Zsig_.col(i);

        MatrixZZ S   = MatrixZZ::Zero();
        MatrixXZ Cxz = MatrixXZ::Zero();

        for (int i = 0; i < N_SIGMA; ++i) {
            MatrixZ1 dz = Zsig_.col(i) - z_pri;
            MatrixX1 dx = Xf_.col(i) - x_pri_;
            S.noalias() += Wc_(i) * (dz * dz.transpose());
            Cxz.noalias() += Wc_(i) * (dx * dz.transpose());
        }

        R_ = this->update_R_(z);
        S.noalias() += R_;

        // 对称化
        S = (S + S.transpose()) * 0.5;

        // LDLT 求解增益（避免逆）
        Eigen::LDLT<MatrixZZ> ldlt(S);
        if (ldlt.info() != Eigen::Success) {
            // 轻微抬升保证正定
            S.noalias() += 1e-9 * MatrixZZ::Identity();
            ldlt.compute(S);
        }

        // 马氏距离门限（可选）
        MatrixZ1 innov = z - z_pri;
        if (gating_threshold_ > 0.0) {
            const double maha = innov.transpose() * ldlt.solve(innov);
            if (std::isfinite(maha) && maha > gating_threshold_) {
                // 拒绝更新：返回上一次状态
                return x_post_;
            }
        }

        // K = Cxz * S^{-1}
        K_.noalias() = Cxz * ldlt.solve(MatrixZZ::Identity());

        // 状态与协方差更新
        x_post_.noalias() = x_pri_ + K_ * innov;
        // Joseph 形式近似可进一步稳定，这里用标准式 + 对称化 todo
        P_post_.noalias() = P_pri_ - K_ * S * K_.transpose();
        P_post_           = (P_post_ + P_post_.transpose()) * 0.5;

        return x_post_;
    }

    void setPredictFunc(const PredictFunc& f) noexcept { f_ = f; }
    void setMeasureFunc(const MeasureFunc& h) noexcept { h_ = h; }

    const MatrixX1& state() const noexcept override { return x_post_; }
    const MatrixXX& covariance() const noexcept override { return P_post_; }
    const MatrixXZ& kalmanGain() const noexcept { return K_; } // 便于调试

private:
    // 生成 sigma 点：带正定性保护
    void sigmaPoints(
        const MatrixX1& x, const MatrixXX& P, double c,
        Eigen::Matrix<double, N_X, N_SIGMA>& X) const {
        MatrixXX Pc = P * c;
        // 对称化 + 轻微抬升
        MatrixXX Pcs = (Pc + Pc.transpose()) * 0.5;
        Eigen::LLT<MatrixXX> llt(Pcs);
        if (llt.info() != Eigen::Success) {
            Pcs.noalias() += 1e-9 * MatrixXX::Identity();
            llt.compute(Pcs);
        }
        MatrixXX S = llt.matrixL();
        X.col(0)   = x;
        for (int i = 0; i < N_X; ++i) {
            X.col(i + 1)       = x + S.col(i);
            X.col(i + 1 + N_X) = x - S.col(i);
        }
    }

    void initWeights() {
        lambda_ = alpha_ * alpha_ * (N_X + kappa_) - N_X;
        Wm_.setConstant(1.0 / (2.0 * (N_X + lambda_)));
        Wc_    = Wm_;
        Wm_(0) = lambda_ / (N_X + lambda_);
        Wc_(0) = lambda_ / (N_X + lambda_) + (1.0 - alpha_ * alpha_ + beta_);
        // 预生成一次固定 sigma 点容器
        Xsig_.setZero();
    }

private:
    // 可插拔模型
    PredictFunc f_;
    MeasureFunc h_;

    // UKF 超参数
    double alpha_, beta_, kappa_;
    double lambda_{0.0};
    double gating_threshold_{-1.0}; // 马氏门限，<0 表示关闭

    // 权重
    Eigen::Matrix<double, N_SIGMA, 1> Wm_;
    Eigen::Matrix<double, N_SIGMA, 1> Wc_;

    // 矩阵缓存（避免重复分配）
    MatrixXX Q_{MatrixXX::Zero()};
    MatrixZZ R_{MatrixZZ::Zero()};
    MatrixXZ K_{MatrixXZ::Zero()};
    MatrixXX P_pri_{MatrixXX::Identity()};
    MatrixXX P_post_{MatrixXX::Identity()};
    MatrixX1 x_pri_{MatrixX1::Zero()};
    MatrixX1 x_post_{MatrixX1::Zero()};

    // Sigma 点缓存
    Eigen::Matrix<double, N_X, N_SIGMA> Xsig_{Eigen::Matrix<double, N_X, N_SIGMA>::Zero()};
    Eigen::Matrix<double, N_X, N_SIGMA> Xf_{Eigen::Matrix<double, N_X, N_SIGMA>::Zero()};
    Eigen::Matrix<double, N_Z, N_SIGMA> Zsig_{Eigen::Matrix<double, N_Z, N_SIGMA>::Zero()};
};

} // namespace at
