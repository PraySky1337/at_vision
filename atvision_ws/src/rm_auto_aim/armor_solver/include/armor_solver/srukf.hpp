#pragma once
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <stdexcept>

namespace srukf {

template <typename Scalar>
struct SigmaPointConfig {
    Scalar alpha{static_cast<Scalar>(0.518)}; // 0.517 - 1.932
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

/**
 * 纯估计器 SR-UKF（无控制输入）
 *   Model 需提供：
 *     VecX f(const VecX& x, Scalar dt) const;
 *     VecZ h(const VecX& x)            const;
 *     MatQ Q_sqrt(Scalar dt)           const; // 形状 (NX x nq)，列因子或下三角；常用 nq=NX
 *     MatR R_sqrt()                    const; // 形状 (NZ x nr)，常用 nr=NZ
 *
 * 说明：
 *   - 预测步用真正的 SR 形式（QR 合并 dev 和 Q_sqrt）
 *   - 更新步为避免实现复杂的 chol-downdate，先在协方差域计算
 *       P^+ = P^- - K P_zz K^T
 *     再用 chol_from_ldlt 重新得到平方根 Sx_
 */
template <typename Model, int NX, int NZ, typename Scalar = double>
class SRUKF {
public:
    using VecX  = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ  = Eigen::Matrix<Scalar, NZ, 1>;
    using MatXX = Eigen::Matrix<Scalar, NX, NX>;
    using MatZZ = Eigen::Matrix<Scalar, NZ, NZ>;
    using MatXZ = Eigen::Matrix<Scalar, NX, NZ>;
    using DevX  = Eigen::Matrix<Scalar, NX, Eigen::Dynamic>;
    using DevZ  = Eigen::Matrix<Scalar, NZ, Eigen::Dynamic>;

    explicit SRUKF(Model& model, const VecX& x0, const MatXX& P0, SigmaPointConfig<Scalar> cfg = {})
        : model_(model)
        , cfg_(cfg) {
        x_  = x0;
        Sx_ = detail::chol_from_ldlt<NX, Scalar>(P0);
        init_weights_();
        Xsig_.resize(2 * NX + 1, NX);
        Zsig_.resize(2 * NX + 1, NZ);
    }

    // 预测
    void predict(Scalar dt) {
        compute_sigma_points_();

        for (int i = 0; i < Xsig_.rows(); ++i) {
            Xsig_.row(i) = model_.f(Xsig_.row(i).transpose(), dt).transpose();
        }

        x_.setZero();
        for (int i = 0; i < Xsig_.rows(); ++i)
            x_.noalias() += wm_(i) * Xsig_.row(i).transpose();

        DevX dev_x;
        dev_x.resize(NX, 2 * NX + 1);
        for (int i = 0; i < Xsig_.rows(); ++i)
            dev_x.col(i) = wc_sqrt_(i) * (Xsig_.row(i).transpose() - x_);

        const auto Qs = model_.Q_sqrt(dt);
        assert(Qs.rows() == NX && "Q_sqrt rows must be NX");
        Sx_ = detail::qr_compose<NX, Scalar>(dev_x, Qs);
    }

    // 更新
    void update(const VecZ& z) {
        compute_sigma_points_();

        for (int i = 0; i < Xsig_.rows(); ++i)
            Zsig_.row(i) = model_.h(Xsig_.row(i).transpose()).transpose();

        VecZ z_pred = VecZ::Zero();
        for (int i = 0; i < Zsig_.rows(); ++i)
            z_pred.noalias() += wm_(i) * Zsig_.row(i).transpose();

        DevZ dev_z;
        dev_z.resize(NZ, 2 * NX + 1);
        for (int i = 0; i < Zsig_.rows(); ++i)
            dev_z.col(i) = wc_sqrt_(i) * (Zsig_.row(i).transpose() - z_pred);

        const auto Rs = model_.R_sqrt(z);
        assert(Rs.rows() == NZ && "R_sqrt rows must be NZ");
        MatZZ Sz = detail::qr_compose<NZ, Scalar>(dev_z, Rs);

        MatXZ Pxz = MatXZ::Zero();
        for (int i = 0; i < Xsig_.rows(); ++i) {
            const VecX dx = Xsig_.row(i).transpose() - x_;
            const VecZ dz = Zsig_.row(i).transpose() - z_pred;
            Pxz.noalias() += wc_(i) * (dx * dz.transpose());
        }

        // K = P_xz * P_zz^{-1}，P_zz = Sz Sz^T
        MatXZ K = detail::solve_via_chol<NZ, NX, Scalar>(Sz, Pxz.transpose()).transpose();

        const VecZ y = z - z_pred;
        x_.noalias() += K * y;

        // 在协方差域更新：P^+ = P^- - K P_zz K^T
        // P^- = Sx_ Sx_^T
        MatXX P_prior = Sx_ * Sx_.transpose();
        MatZZ Pzz     = Sz * Sz.transpose();
        MatXX P_post  = P_prior - K * Pzz * K.transpose();

        // 重新分解得到平方根（带 jitter 的 LDLT -> LLT），保持对称正定
        Sx_ = detail::chol_from_ldlt<NX, Scalar>(P_post);
    }

    // predict + update
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
    SigmaPointConfig<Scalar> cfg_;

    VecX x_;
    MatXX Sx_;

    Scalar lambda_{};
    Scalar gamma_{};
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> wm_, wc_, wc_sqrt_;

    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Xsig_; // (2n+1 x NX)
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Zsig_; // (2n+1 x NZ)

    void init_weights_() {
        const Scalar n = static_cast<Scalar>(NX);
        lambda_        = cfg_.alpha * cfg_.alpha * (n + cfg_.kappa) - n;
        // 保证 n + lambda 不接近 0，避免权重无穷大
        const Scalar nlam_min = static_cast<Scalar>(1e-6);
        if (n + lambda_ <= nlam_min)
            lambda_ = nlam_min - n;
        gamma_ = std::sqrt(n + lambda_);

        wm_.setConstant(2 * NX + 1, Scalar(1) / (2 * (n + lambda_)));
        wc_    = wm_;
        wm_(0) = lambda_ / (n + lambda_);
        wc_(0) = wm_(0) + (Scalar(1) - cfg_.alpha * cfg_.alpha + cfg_.beta);

        // 要求协方差权重非负（至少不显著为负），否则当前 SR 实现不适用
        Scalar min_wc = wc_.minCoeff();
        if (min_wc < Scalar(-1e-12)) {
            throw std::runtime_error(
                "SRUKF: covariance weights must be non-negative; adjust alpha/beta/kappa.");
        }

        wc_sqrt_.resizeLike(wc_);
        for (int i = 0; i < wc_.size(); ++i)
            wc_sqrt_(i) = std::sqrt(std::max(wc_(i), Scalar(0))); // 允许极小负值被截断
    }

    void compute_sigma_points_() {
        Xsig_.row(0)                    = x_.transpose();
        Eigen::Matrix<Scalar, NX, NX> A = Sx_ * gamma_;
        for (int j = 0; j < NX; ++j) {
            Xsig_.row(1 + j)      = (x_ + A.col(j)).transpose();
            Xsig_.row(1 + NX + j) = (x_ - A.col(j)).transpose();
        }
    }
};

} // namespace srukf
