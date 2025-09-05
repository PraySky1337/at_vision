#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace trajectory {

class ADMM_QP {
public:
    using Mat = Eigen::MatrixXd;
    using Vec = Eigen::VectorXd;

    struct Result {
        Vec z;
        int iters    = 0;
        double r_pri = 0, r_dual = 0;
        bool converged = false;
    };

    // ① 默认构造：占位，需后续 set_problem()
    ADMM_QP()
        : rho_(1.0)
        , max_iter_(200)
        , eps_abs_(1e-4)
        , eps_rel_(1e-4)
        , ready_(false) {}

    // 仍保留原有有参构造
    ADMM_QP(
        const Mat& H, const Vec& f, const Vec& lb, const Vec& ub, double rho = 1.0,
        int max_iter = 200, double eps_abs = 1e-4, double eps_rel = 1e-4)
        : H_(symmetrize(H))
        , f_(f)
        , lb_(lb)
        , ub_(ub)
        , rho_(rho)
        , max_iter_(max_iter)
        , eps_abs_(eps_abs)
        , eps_rel_(eps_rel)
        , ready_(false) {
        assert_dims();
        factorize();
    }

    // 更新问题
    void set_problem(const Mat& H, const Vec& f, const Vec& lb, const Vec& ub) {
        H_  = symmetrize(H);
        f_  = f;
        lb_ = lb;
        ub_ = ub;
        assert_dims();
        factorize();
    }

    void set_rho(double rho) {
        rho_ = rho;
        if (ready_)
            factorize();
    }
    void set_stopping(int max_iter, double eps_abs, double eps_rel) {
        max_iter_ = max_iter;
        eps_abs_  = eps_abs;
        eps_rel_  = eps_rel;
    }

    Result solve(const Vec* warm = nullptr) {
        if (!ready_)
            throw std::logic_error("ADMM_QP: problem not set (call set_problem)");
        const int n = f_.size();
        Vec x       = (warm && warm->size() == n) ? *warm : Vec::Zero(n);
        Vec z       = project_box(x, lb_, ub_);
        Vec u       = Vec::Zero(n);

        Result R;
        for (int k = 0; k < max_iter_; ++k) {
            Vec rhs = -f_ + rho_ * (z - u);
            x       = llt_.solve(rhs);

            Vec z_old = z;
            z         = project_box(x + u, lb_, ub_);

            u.noalias() += x - z;

            const double r_pri  = (x - z).norm();
            const double r_dual = (rho_ * (z - z_old)).norm();
            const double eps_pri =
                std::sqrt((double)n) * eps_abs_ + eps_rel_ * std::max(x.norm(), z.norm());
            const double eps_dual = std::sqrt((double)n) * eps_abs_ + eps_rel_ * (rho_ * u).norm();

            R.iters  = k + 1;
            R.r_pri  = r_pri;
            R.r_dual = r_dual;
            if (r_pri <= eps_pri && r_dual <= eps_dual) {
                R.converged = true;
                break;
            }
        }
        R.z = std::move(z);
        return R;
    }

    // —— 以下为“特化到云台加速度盒约束 QP”的便捷工厂 —— //

    // ② 单步（贪心一拍）a_cmd：min (θ_k+Tω_k+0.5T^2 a - θ*)^2 + λ a^2, s.t. |a|<=a_max
    static ADMM_QP for_single_step(
        double theta_k, double omega_k, double theta_ref, double T, double lambda, double a_max,
        double rho = 1.0) {
        const double c  = 0.5 * T * T;            // 对 a 的系数
        const double Hs = 2.0 * (c * c + lambda); // 标量 H
        const double fs = 2.0 * (theta_k + T * omega_k - theta_ref) * c;

        Mat H(1, 1);
        H(0, 0) = Hs;
        Vec f(1);
        f(0) = fs;
        Vec lb(1), ub(1);
        lb(0) = -a_max;
        ub(0) = a_max;

        ADMM_QP s(H, f, lb, ub, rho);
        return s;
    }

    // ③ N 步 MPC：x=[a0..aN-1],  min ||A x - b||^2 + λ||x||^2,  s.t. |a_i|<=a_max
    // theta_ref: N 维 [θ*_{k+1}..θ*_{k+N}]
    static ADMM_QP for_mpc(
        int N, double T, double lambda, double a_max, double theta_k, double omega_k,
        const Vec& theta_ref, double rho = 1.0) {
        if (theta_ref.size() != N)
            throw std::invalid_argument("theta_ref size != N");

        Mat A = Mat::Zero(N, N);
        for (int i = 0; i < N; i++)
            for (int j = 0; j <= i; j++)
                A(i, j) = 0.5 * T * T * ((i - j) + 1); // 见推导

        Vec d(N);
        for (int i = 0; i < N; i++)
            d(i) = theta_k + (i + 1) * T * omega_k;

        Vec b = theta_ref - d;

        Mat H = 2.0 * (A.transpose() * A + lambda * Mat::Identity(N, N));
        Vec f = -2.0 * A.transpose() * b;

        Vec lb = Vec::Constant(N, -a_max);
        Vec ub = Vec::Constant(N, a_max);

        ADMM_QP s(H, f, lb, ub, rho);
        return s;
    }

    // 可选：带加速度“平滑”项 λ||D x||^2
    static ADMM_QP for_mpc_with_smoothing(
        int N, double T, double lambda_D, double a_max, double theta_k, double omega_k,
        const Vec& theta_ref, double rho = 1.0) {
        if (theta_ref.size() != N)
            throw std::invalid_argument("theta_ref size != N");

        Mat A = Mat::Zero(N, N);
        for (int i = 0; i < N; i++)
            for (int j = 0; j <= i; j++)
                A(i, j) = 0.5 * T * T * ((i - j) + 1);

        Vec d(N);
        for (int i = 0; i < N; i++)
            d(i) = theta_k + (i + 1) * T * omega_k;

        Vec b = theta_ref - d;

        // 一阶差分矩阵 D ∈ R^{(N-1)×N}
        Mat D = Mat::Zero(N - 1, N);
        for (int i = 0; i < N - 1; i++) {
            D(i, i)     = -1.0;
            D(i, i + 1) = 1.0;
        }

        Mat H = 2.0 * (A.transpose() * A + lambda_D * D.transpose() * D);
        Vec f = -2.0 * A.transpose() * b;

        Vec lb = Vec::Constant(N, -a_max);
        Vec ub = Vec::Constant(N, a_max);

        ADMM_QP s(H, f, lb, ub, rho);
        return s;
    }

private:
    Mat H_;
    Vec f_, lb_, ub_;
    double rho_;
    int max_iter_;
    double eps_abs_, eps_rel_;

    Mat K_;
    Eigen::LLT<Mat> llt_;
    Eigen::MatrixXd L_, LT_;
    bool ready_; // ← 新增：问题是否已分解就绪

    static Mat symmetrize(const Mat& H) { return 0.5 * (H + H.transpose()); }

    static Vec project_box(const Vec& v, const Vec& lb, const Vec& ub) {
        Vec z = v;
        for (int i = 0; i < z.size(); ++i) {
            if (std::isfinite(lb[i]))
                z[i] = std::max(z[i], lb[i]);
            if (std::isfinite(ub[i]))
                z[i] = std::min(z[i], ub[i]);
        }
        return z;
    }

    void assert_dims() const {
        const int n = (int)f_.size();
        if (H_.rows() != n || H_.cols() != n)
            throw std::invalid_argument("H size mismatch");
        if (lb_.size() != n || ub_.size() != n)
            throw std::invalid_argument("lb/ub size mismatch");
    }

    void factorize() {
        const int n = (int)f_.size();
        K_          = H_ + rho_ * Mat::Identity(n, n);
        llt_.compute(K_);
        if (llt_.info() != Eigen::Success) {
            K_.diagonal().array() += 1e-9;
            llt_.compute(K_);
            if (llt_.info() != Eigen::Success)
                throw std::runtime_error("Cholesky failed: try larger rho or regularize H.");
        }
        L_     = llt_.matrixL();
        LT_    = L_.transpose();
        ready_ = true;
    }
};

} // namespace trajectory
