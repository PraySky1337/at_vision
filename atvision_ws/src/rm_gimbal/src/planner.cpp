#include "rm_gimbal/planner.hpp"
#include <algorithm>
#include <chrono>

namespace rm_gimbal {

void Planner::destroySolver(TinySolver*& s) {
    // 若 TinyMPC 的版本提供销毁 API，请在此调用。
    s = nullptr;
}

void Planner::buildOneSolver(
    TinySolver*& solver, int N, double dt, double amax, double q_pos, double q_vel, double r_acc) {
    destroySolver(solver);

    // x=[θ, ω], u=[a]
    Eigen::MatrixXd A(2, 2);
    A << 1.0, dt, 0.0, 1.0;
    Eigen::MatrixXd B(2, 1);
    B << 0.0, dt;
    Eigen::Vector2d f = Eigen::Vector2d::Zero();

    Eigen::Matrix2d Q = Eigen::Vector2d(q_pos, q_vel).asDiagonal();
    Eigen::Matrix<double, 1, 1> R;
    R(0, 0) = r_acc;

    tiny_setup(&solver, A, B, f, Q, R, 1.0, 2, 1, N, 0);

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, N, -1e18);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, N, 1e18);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, N - 1, -amax);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, N - 1, amax);
    tiny_set_bound_constraints(solver, x_min, x_max, u_min, u_max);

    solver->settings->max_iter = 10;
}

void Planner::rebuildSolvers(double dt) {
    buildOneSolver(yaw_solver_, N_, dt, amax_, q_yaw_pos_, q_yaw_vel_, r_yaw_acc_);
    buildOneSolver(pitch_solver_, N_, dt, amax_, q_pitch_pos_, q_pitch_vel_, r_pitch_acc_);
}

// ---------- 构造/析构 ----------
Planner::Planner(rclcpp::Node::WeakPtr node)
    : node_(node) {
    // 参数声明（与原版一致）
    auto n        = node_.lock();
    N_            = n->declare_parameter<int>("planner.N", 20);
    T_            = n->declare_parameter<double>("planner.T", 0.01);
    amax_         = n->declare_parameter<double>("planner.amax", 50.0);
    dt_min_       = n->declare_parameter<double>("planner.dt_min", 0.002);
    dt_max_       = n->declare_parameter<double>("planner.dt_max", 0.03);
    dt_ema_alpha_ = n->declare_parameter<double>("planner.dt_ema_alpha", 0.2);

    q_yaw_pos_ = n->declare_parameter<double>("planner.q_yaw_pos", 9e6);
    q_yaw_vel_ = n->declare_parameter<double>("planner.q_yaw_vel", 0.);
    r_yaw_acc_ = n->declare_parameter<double>("planner.r_yaw_acc", 1.);

    q_pitch_pos_ = n->declare_parameter<double>("planner.q_pitch_pos", 9e6);
    q_pitch_vel_ = n->declare_parameter<double>("planner.q_pitch_vel", 0.);
    r_pitch_acc_ = n->declare_parameter<double>("planner.r_pitch_acc", 1.);

    last_dt_ = T_;

    // 构建 TinyMPC
    rebuildSolvers(T_);

    // 参数热更新（挂在注入的 node 上）
    param_cb_handle_ = n->add_on_set_parameters_callback(
        std::bind(&Planner::onParam, this, std::placeholders::_1));
}

Planner::~Planner() {
    destroySolver(yaw_solver_);
    destroySolver(pitch_solver_);
}

void Planner::reset(double yaw, double yaw_rate, double pitch, double pitch_rate) {
    std::scoped_lock lk(param_mtx_);
    yaw_hat_        = yaw;
    yaw_rate_hat_   = yaw_rate;
    pitch_hat_      = pitch;
    pitch_rate_hat_ = pitch_rate;
    have_last_now_  = false;
    last_dt_        = T_;
}

// ---------- 参数回调 ----------
rcl_interfaces::msg::SetParametersResult
    Planner::onParam(const std::vector<rclcpp::Parameter>& ps) {
    int N    = N_;
    double T = T_, amax = amax_;
    double dt_min = dt_min_, dt_max = dt_max_, alpha = dt_ema_alpha_;
    double qyp = q_yaw_pos_, qyv = q_yaw_vel_, rya = r_yaw_acc_;
    double qpp = q_pitch_pos_, qpv = q_pitch_vel_, rpa = r_pitch_acc_;

    for (const auto& p : ps) {
        const auto& n = p.get_name();
        if (n == "planner.N")
            N = p.as_int();
        else if (n == "planner.T")
            T = p.as_double();
        else if (n == "planner.amax")
            amax = p.as_double();
        else if (n == "planner.dt_min")
            dt_min = p.as_double();
        else if (n == "planner.dt_max")
            dt_max = p.as_double();
        else if (n == "planner.dt_ema_alpha")
            alpha = p.as_double();
        else if (n == "planner.q_yaw_pos")
            qyp = p.as_double();
        else if (n == "planner.q_yaw_vel")
            qyv = p.as_double();
        else if (n == "planner.r_yaw_acc")
            rya = p.as_double();
        else if (n == "planner.q_pitch_pos")
            qpp = p.as_double();
        else if (n == "planner.q_pitch_vel")
            qpv = p.as_double();
        else if (n == "planner.r_pitch_acc")
            rpa = p.as_double();
    }

    rcl_interfaces::msg::SetParametersResult res;

    if (N <= 1) {
        res.successful = false;
        res.reason     = "N must be > 1";
        return res;
    }
    if (T <= 0.0) {
        res.successful = false;
        res.reason     = "T must be > 0";
        return res;
    }
    if (amax <= 0.0) {
        res.successful = false;
        res.reason     = "amax must be > 0";
        return res;
    }
    if (dt_min <= 0.0) {
        res.successful = false;
        res.reason     = "dt_min must be > 0";
        return res;
    }
    if (dt_max < dt_min) {
        res.successful = false;
        res.reason     = "dt_max must be >= dt_min";
        return res;
    }

    bool need_rebuild = (N != N_) || std::abs(T - T_) > 1e-9 || std::abs(amax - amax_) > 1e-9
                     || qyp != q_yaw_pos_ || qyv != q_yaw_vel_ || rya != r_yaw_acc_
                     || qpp != q_pitch_pos_ || qpv != q_pitch_vel_ || rpa != r_pitch_acc_;

    N_            = N;
    T_            = T;
    amax_         = amax;
    dt_min_       = dt_min;
    dt_max_       = dt_max;
    dt_ema_alpha_ = alpha;
    q_yaw_pos_    = qyp;
    q_yaw_vel_    = qyv;
    r_yaw_acc_    = rya;
    q_pitch_pos_  = qpp;
    q_pitch_vel_  = qpv;
    r_pitch_acc_  = rpa;
    last_dt_      = std::clamp(last_dt_, dt_min_, dt_max_);
    if (need_rebuild)
        rebuildSolvers(T_);
    res.successful = true;
    return res;
}

auto Planner::process(const rm_interfaces::msg::GimbalCmd& msg)
    -> rm_interfaces::msg::PlanGimbalCmd {
    auto to_f32 = [](double v) -> float {
        if (!std::isfinite(v))
            return 0.0f;
        constexpr float FMAX = std::numeric_limits<float>::max();
        if (v > FMAX)
            return FMAX;
        if (v < -FMAX)
            return -FMAX;
        return static_cast<float>(v);
    };
    auto now       = std::chrono::steady_clock::now();
    double raw_dt  = have_last_now_ ? std::chrono::duration<double>(now - last_now_).count() : T_;
    last_now_      = now;
    have_last_now_ = true;

    raw_dt    = std::clamp(raw_dt, dt_min_, dt_max_);
    double dt = dt_ema_alpha_ * raw_dt + (1.0 - dt_ema_alpha_) * last_dt_;
    last_dt_  = dt;

    // 2) 参考轨迹：整段目标角 + 0 角速
    const double ty = msg.yaw * M_PI / 180.;
    const double tp = msg.pitch * M_PI / 180.;

    Eigen::MatrixXd Xref_y(2, N_), Xref_p(2, N_);
    Xref_y.row(0).setConstant(ty);
    Xref_y.row(1).setZero();
    Xref_p.row(0).setConstant(tp);
    Xref_p.row(1).setZero();

    Eigen::Vector2d x0y(yaw_hat_, yaw_rate_hat_), x0p(pitch_hat_, pitch_rate_hat_);

    // 4) TinyMPC（两轴独立）
    tiny_set_x0(yaw_solver_, x0y);
    yaw_solver_->work->Xref = Xref_y;
    tiny_solve(yaw_solver_);

    tiny_set_x0(pitch_solver_, x0p);
    pitch_solver_->work->Xref = Xref_p;
    tiny_solve(pitch_solver_);

    // 5) 首个控制 + 一步状态 + 更新先验
    const double a_yaw   = std::clamp(yaw_solver_->work->u(0, 0), -amax_, amax_);
    const double a_pitch = std::clamp(pitch_solver_->work->u(0, 0), -amax_, amax_);

    yaw_hat_        = yaw_solver_->work->x(0, 1);
    yaw_rate_hat_   = yaw_solver_->work->x(1, 1);
    pitch_hat_      = pitch_solver_->work->x(0, 1);
    pitch_rate_hat_ = pitch_solver_->work->x(1, 1);

    // 6) 返回结果（由外部发布）
    rm_interfaces::msg::PlanGimbalCmd out;
    out.ref_yaw       = to_f32(yaw_hat_ * 180.0 / M_PI);
    out.ref_yaw_vel   = to_f32(yaw_rate_hat_ * 180.0 / M_PI);
    out.ref_yaw_acc   = to_f32(a_yaw * 180.0 / M_PI);
    out.ref_pitch     = to_f32(pitch_hat_ * 180.0 / M_PI);
    out.ref_pitch_vel = to_f32(pitch_rate_hat_ * 180.0 / M_PI);
    out.ref_pitch_acc = to_f32(a_pitch * 180.0 / M_PI);
    return out;
}

} // namespace rm_gimbal