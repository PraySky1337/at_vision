#pragma once
#include "srukf.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <cassert>

namespace srukf {

/**
 * Iterated Square-Root Cubature Kalman Filter (ISR-SRCKF)
 * - 2N cubature 点（权重全正），避免 UKF 权重为负导致 SR 实现不适配的问题
 * - 量测更新用 Gauss-Newton / MAP 形式迭代（统计线性回归近似雅可比）
 * - public 接口风格一致，可以在ISRCKF和SRUKF之间无缝切换：predict/update/step/x/P/Sx
 *
 * Model 需提供（与 SRUKF 一致）：
 *   VecX f(const VecX& x, Scalar dt) const;
 *   VecZ h(const VecX& x) const;
 *   MatQ Q_sqrt(Scalar dt) const;           // (NX x nq)，常用 nq=NX
 *   MatR R_sqrt(const VecZ& z) const;       // (NZ x nr)，常用 nr=NZ
 */
template <typename Model, int NX, int NZ, typename Scalar = double>
class ISRCKF {
public:
    using VecX  = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ  = Eigen::Matrix<Scalar, NZ, 1>;
    using MatXX = Eigen::Matrix<Scalar, NX, NX>;
    using MatZZ = Eigen::Matrix<Scalar, NZ, NZ>;
    using MatXZ = Eigen::Matrix<Scalar, NX, NZ>;
    using DevX  = Eigen::Matrix<Scalar, NX, Eigen::Dynamic>;
    using DevZ  = Eigen::Matrix<Scalar, NZ, Eigen::Dynamic>;

    explicit ISRCKF(Model& model,
                    const VecX& x0,
                    const MatXX& P0,
                    SigmaPointConfig<Scalar> /*cfg_compat*/ = {})
        : model_(model) {
        x_  = x0;
        Sx_ = detail::chol_from_ldlt<NX, Scalar>(P0);
        gamma_   = std::sqrt(static_cast<Scalar>(NX));       // CKF 固定
        w_       = Scalar(1) / (Scalar(2) * Scalar(NX));     // 2N 点，权重相等
        w_sqrt_  = std::sqrt(w_);

        Xsig_.resize(2 * NX, NX);
        Zsig_.resize(2 * NX, NZ);
    }

    // 可选：调迭代次数与收敛阈值（不影响旧代码编译）
    void set_max_iters(int iters) { max_iters_ = (iters < 1 ? 1 : iters); }
    void set_iter_tol(Scalar tol) { iter_tol_ = (tol <= Scalar(0) ? Scalar(1e-6) : tol); }

    void predict(Scalar dt) {
        compute_cubature_points_(x_, Sx_, Xsig_);

        // 传播
        for (int i = 0; i < Xsig_.rows(); ++i) {
            Xsig_.row(i) = model_.f(Xsig_.row(i).transpose(), dt).transpose();
        }

        // 均值
        VecX x_pred = VecX::Zero();
        for (int i = 0; i < Xsig_.rows(); ++i) x_pred.noalias() += w_ * Xsig_.row(i).transpose();

        // SR 协方差
        DevX dev_x(NX, Xsig_.rows());
        for (int i = 0; i < Xsig_.rows(); ++i)
            dev_x.col(i) = w_sqrt_ * (Xsig_.row(i).transpose() - x_pred);

        const auto Qs = model_.Q_sqrt(dt);
        assert(Qs.rows() == NX && "Q_sqrt rows must be NX");
        Sx_ = detail::qr_compose<NX, Scalar>(dev_x, Qs);

        x_ = x_pred;
    }

    void update(const VecZ& z) {
        // prior 固定（迭代时不改变先验协方差）
        const VecX  x_prior = x_;
        const MatXX S_prior = Sx_;
        const MatXX P_prior = S_prior * S_prior.transpose();

        VecX x_it = x_prior;

        // 迭代中需要缓存最后一次的量测统计量，用于最终协方差更新
        MatXZ Pxz_last = MatXZ::Zero();
        MatZZ Sz_last  = MatZZ::Zero();
        VecZ  zpred_last = VecZ::Zero();

        for (int it = 0; it < max_iters_; ++it) {
            compute_cubature_points_(x_it, S_prior, Xsig_);

            // 量测传播
            for (int i = 0; i < Xsig_.rows(); ++i)
                Zsig_.row(i) = model_.h(Xsig_.row(i).transpose()).transpose();

            // 量测均值
            VecZ z_pred = VecZ::Zero();
            for (int i = 0; i < Zsig_.rows(); ++i) z_pred.noalias() += w_ * Zsig_.row(i).transpose();

            // dev_z（加 R_sqrt）
            DevZ dev_z(NZ, Zsig_.rows());
            for (int i = 0; i < Zsig_.rows(); ++i)
                dev_z.col(i) = w_sqrt_ * (Zsig_.row(i).transpose() - z_pred);

            const auto Rs = model_.R_sqrt(z);
            assert(Rs.rows() == NZ && "R_sqrt rows must be NZ");
            MatZZ Sz = detail::qr_compose<NZ, Scalar>(dev_z, Rs);

            // Pxz：围绕 x_it 做统计线性回归（近似 H(x_it)）
            MatXZ Pxz = MatXZ::Zero();
            for (int i = 0; i < Xsig_.rows(); ++i) {
                const VecX dx = Xsig_.row(i).transpose() - x_it;
                const VecZ dz = Zsig_.row(i).transpose() - z_pred;
                Pxz.noalias() += w_ * (dx * dz.transpose());
            }

            // K = P_xz * P_zz^{-1}, P_zz = Sz Sz^T
            MatXZ K = detail::solve_via_chol<NZ, NX, Scalar>(Sz, Pxz.transpose()).transpose();

            // Gauss-Newton / MAP 迭代项：z - h(x_it) + H(x_it)*(x_it - x_prior)
            // 其中 H(x_it) ≈ P_xz^T * P_prior^{-1}
            const VecX dxp = x_it - x_prior;
            const VecX v   = detail::solve_via_chol<NX, 1, Scalar>(S_prior, dxp);          // v = P_prior^{-1} * (x_it-x_prior)
            const VecZ Hdx = Pxz.transpose() * v;                                          // ≈ H(x_it)*(x_it-x_prior)

            const VecZ innov = (z - z_pred) + Hdx;
            const VecX x_new = x_prior + K * innov;

            // 保存最后一次统计量
            Pxz_last   = Pxz;
            Sz_last    = Sz;
            zpred_last = z_pred;

            const Scalar step_norm = (x_new - x_it).template lpNorm<Eigen::Infinity>();
            x_it = x_new;
            if (step_norm < iter_tol_) break;
        }

        // 状态更新
        x_ = x_it;

        // 协方差更新（仍在协方差域做一次，最后重分解回 SR）
        const MatZZ Pzz = Sz_last * Sz_last.transpose();
        MatXZ K_last = detail::solve_via_chol<NZ, NX, Scalar>(Sz_last, Pxz_last.transpose()).transpose();

        MatXX P_post = P_prior - K_last * Pzz * K_last.transpose();
        Sx_ = detail::chol_from_ldlt<NX, Scalar>(P_post);
    }

    void step(const VecZ& z, Scalar dt) {
        predict(dt);
        update(z);
    }

    const VecX& x() const noexcept { return x_; }
    VecX* x_raw_ptr() noexcept { return &x_; }
    MatXX P() const { return Sx_ * Sx_.transpose(); }
    const MatXX& Sx() const noexcept { return Sx_; }

private:
    Model& model_;

    VecX  x_{};
    MatXX Sx_{};

    Scalar gamma_{};
    Scalar w_{};
    Scalar w_sqrt_{};

    int    max_iters_{3};                 // 默认 3 次迭代
    Scalar iter_tol_{Scalar(1e-6)};       // 默认无穷范数阈值

    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Xsig_; // (2N x NX)
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Zsig_; // (2N x NZ)

    void compute_cubature_points_(const VecX& xc,
                                  const MatXX& S,
                                  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& Xsig) {
        Eigen::Matrix<Scalar, NX, NX> A = S * gamma_;
        for (int j = 0; j < NX; ++j) {
            Xsig.row(j)      = (xc + A.col(j)).transpose();
            Xsig.row(NX + j) = (xc - A.col(j)).transpose();
        }
    }
};

} // namespace srukf
