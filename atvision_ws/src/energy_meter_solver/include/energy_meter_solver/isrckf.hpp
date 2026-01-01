#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <cassert>

template <typename Scalar>
struct SigmaPointConfig {
    Scalar alpha{static_cast<Scalar>(0.7)}; // 0.517 - 1.932
    Scalar beta{static_cast<Scalar>(2.0)};
    Scalar kappa{static_cast<Scalar>(0)};
};

namespace detail {

// 由 LDLT 产生“可用于三角回代”的下三角平方根因子：
// 1) 尝试 LDLT（带置换）。若置换为单位且 D>0：直接构造 S = L * sqrt(D)
// 2) 否则：按 LDLT 的最小负值自适应加入 jitter，改用 LLT 得到 S（仍为下三角）
// 这样：既利用了 LDLT 的稳健性，又保持 SR-UKF 的三角结构不变。
template <int N, typename Scalar>
inline Eigen::Matrix<Scalar, N, N> chol_from_ldlt(
    const Eigen::Matrix<Scalar, N, N>& Pin, Scalar base_jitter = Scalar(1e-12), int max_try = 6) {
    using Mat = Eigen::Matrix<Scalar, N, N>;
    Mat P     = (Pin + Pin.transpose()) * Scalar(0.5); // 对称化

    // 1) LDLT（带置换）
    Eigen::LDLT<Mat> ldlt(P);
    const bool ok     = (ldlt.info() == Eigen::Success);
    const auto& trans = ldlt.transpositionsP();
    bool no_perm      = true;
    for (int i = 0; i < N; ++i) {
        if (trans.indices()(i) != i) {
            no_perm = false;
            break;
        }
    }
    Eigen::Matrix<Scalar, N, 1> D;
    if (ok)
        D = ldlt.vectorD();

    // 若无置换且 D>0：可直接构造 S = L*sqrt(D)
    if (ok && no_perm) {
        bool all_pos = true;
        for (int i = 0; i < N; ++i)
            if (!(D(i) > Scalar(0))) {
                all_pos = false;
                break;
            }
        if (all_pos) {
            Mat L = ldlt.matrixL(); // 单位对角的下三角
            Mat S = L;              // 右乘 sqrt(D)（对角）
            for (int i = 0; i < N; ++i)
                S.col(i) *= std::sqrt(D(i));
            S.template triangularView<Eigen::StrictlyUpper>().setZero();
            // 归一化对角符号（翻转列，保持 P = S S^T 不变）
            for (int i = 0; i < N; ++i)
                if (S(i, i) < Scalar(0))
                    S.col(i) = -S.col(i);
            return S;
        }
    }

    // 2) 需要置换或 D 非正：依据 LDLT 的最小特征提示加 jitter，然后用 LLT 得到正定的 Cholesky
    Scalar jitter = base_jitter;
    Scalar min_d  = Scalar(0);
    if (ok) {
        min_d = D.minCoeff();
        if (min_d <= Scalar(0)) {
            // 以 |min_d| 为尺度的抖动起点
            jitter = std::max(jitter, Scalar(1e-12) + Scalar(1e-3) * std::abs(min_d));
        }
    }
    // 多次退避增加 jitter
    for (int t = 0; t < max_try; ++t) {
        Mat Pj = P;
        Pj.diagonal().array() += jitter;
        Eigen::LLT<Mat> llt(Pj);
        if (llt.info() == Eigen::Success) {
            Mat S = llt.matrixL();
            // 标准化符号（翻转列）
            for (int i = 0; i < N; ++i)
                if (S(i, i) < Scalar(0))
                    S.col(i) = -S.col(i);
            return S;
        }
        jitter *= Scalar(10);
    }
    // 仍然失败：保底（会触发断言）
    Eigen::LLT<Mat> llt(P);
    assert(llt.info() == Eigen::Success && "P not SPD even after jitter");
    Mat S = llt.matrixL();
    for (int i = 0; i < N; ++i)
        if (S(i, i) < Scalar(0))
            S.col(i) = -S.col(i);
    return S;
}

// 合成平方根：S S^T = dev dev^T + noise_sqrt noise_sqrt^T
template <int D, typename Scalar>
inline Eigen::Matrix<Scalar, D, D> qr_compose(
    const Eigen::Ref<const Eigen::Matrix<Scalar, D, Eigen::Dynamic>>& dev,
    const Eigen::Ref<const Eigen::Matrix<Scalar, D, Eigen::Dynamic>>& noise_sqrt) {
    const int k = static_cast<int>(dev.cols());
    const int m = static_cast<int>(noise_sqrt.cols());
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MT(k + m, D);
    if (k)
        MT.topRows(k) = dev.transpose();
    if (m)
        MT.bottomRows(m) = noise_sqrt.transpose();

    Eigen::HouseholderQR<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> qr(MT);
    Eigen::Matrix<Scalar, D, D> R = qr.matrixQR().template topLeftCorner<D, D>();
    R.template triangularView<Eigen::StrictlyLower>().setZero();

    Eigen::Matrix<Scalar, D, D> S = R.transpose();
    S.template triangularView<Eigen::StrictlyUpper>().setZero();
    // 对角符号归一化（翻列）
    for (int i = 0; i < D; ++i)
        if (S(i, i) < Scalar(0))
            S.col(i) = -S.col(i);
    return S;
}

// 解 (S S^T) X = B
template <int D, int C, typename Scalar>
inline Eigen::Matrix<Scalar, D, C>
    solve_via_chol(const Eigen::Matrix<Scalar, D, D>& S, const Eigen::Matrix<Scalar, D, C>& B) {
    Eigen::Matrix<Scalar, D, C> Y = S.template triangularView<Eigen::Lower>().solve(B);
    Eigen::Matrix<Scalar, D, C> X = S.transpose().template triangularView<Eigen::Upper>().solve(Y);
    return X;
}

} // namespace detail

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
