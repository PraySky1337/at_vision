#include "trajectory/planner_node.hpp"
#include <algorithm>
#include <rclcpp/qos.hpp>

namespace trajectory {

Planner::Planner(const rclcpp::NodeOptions& options)
    : rclcpp::Node("planner", options) {
    // 参数

    N_        = this->declare_parameter<int>("N", 20);
    T_        = this->declare_parameter<double>("T", 0.01);
    amax_     = this->declare_parameter<double>("amax", 300.0);
    lambda_D_ = this->declare_parameter<double>("lambda_D", 1e-4);
    rho_      = this->declare_parameter<double>("rho", 3.0); // 惩罚参数

    // auto qos    = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto qos              = rclcpp::SensorDataQoS();
    plan_control_cmd_pub_ = this->create_publisher<PlannedControlCmd>("planner/control_cmd", qos);
    target_sub_           = this->create_subscription<ControlCmd>(
        "trajectory/control_cmd", qos, std::bind(&Planner::onTarget, this, std::placeholders::_1));

    // 定时控制环（MPC 仅做规划）
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(T_), std::bind(&Planner::controlStep, this));
}

void Planner::onTarget(const ControlCmd::SharedPtr msg) {
    tgt_yaw_.store(msg->target_yaw, std::memory_order_relaxed);
    tgt_pitch_.store(msg->target_pitch, std::memory_order_relaxed);
    has_tgt_.store(true, std::memory_order_release);
}

void Planner::controlStep() {
    if (!has_tgt_.load())
        return;

    const double ty = tgt_yaw_.load(), tp = tgt_pitch_.load();

    // 参考常值（也可以换成随时间的 ref 向量）
    Eigen::VectorXd ref_yaw   = Eigen::VectorXd::Constant(N_, ty);
    Eigen::VectorXd ref_pitch = Eigen::VectorXd::Constant(N_, tp);

    // MPC 解：返回加速度序列 z（控制量）
    auto sy = ADMM_QP::for_mpc_with_smoothing(
        N_, T_, lambda_D_, amax_, yaw_hat_, yaw_rate_hat_, ref_yaw, rho_);
    auto sp = ADMM_QP::for_mpc_with_smoothing(
        N_, T_, lambda_D_, amax_, pitch_hat_, pitch_rate_hat_, ref_pitch, rho_);
    auto Ry     = sy.solve((warm_yaw_.size() == N_) ? &warm_yaw_ : nullptr);
    auto Rp     = sp.solve((warm_pitch_.size() == N_) ? &warm_pitch_ : nullptr);
    warm_yaw_   = Ry.z;
    warm_pitch_ = Rp.z;

    // —— 选拍：按总时延设置 d —— //
    int d          = 0; // 如果测得 ~1T 时延，就设 1；以此类推
    auto clampA    = [&](double a) { return std::clamp(a, -amax_, amax_); };
    double a_yaw   = clampA(Ry.z(d));
    double a_pitch = clampA(Rp.z(d));

    // === 生成 (x_ref, v_ref, a_ref) 三元组 ===
    // 方案A：d == 0（无预览），直接用当前估计 + 本拍加速度
    double xref_y = yaw_hat_;
    double vref_y = yaw_rate_hat_;
    double aref_y = a_yaw;

    double xref_p = pitch_hat_;
    double vref_p = pitch_rate_hat_;
    double aref_p = a_pitch;

    // 若 d > 0：把 (x,v) 用 Ry.z 的前 d 项推进 d 步（常加速度离散积分）
    if (d > 0) {
        auto advance = [&](double x0, double v0, const Eigen::VectorXd& a, int d) {
            double x = x0, v = v0;
            for (int i = 0; i < d; ++i) { // ZOH, a[i] 作用一个采样周期
                v += a[i] * T_;
                x += v * T_;
            }
            return std::pair<double, double>(x, v);
        };
        std::tie(xref_y, vref_y) = advance(yaw_hat_, yaw_rate_hat_, Ry.z, d);
        std::tie(xref_p, vref_p) = advance(pitch_hat_, pitch_rate_hat_, Rp.z, d);
        aref_y                   = clampA(Ry.z(d));
        aref_p                   = clampA(Rp.z(d));
    }

    // === 发布给控制器（位置环/速度环/电流环可分别用） ===
    PlannedControlCmd out;
    out.header.stamp  = now();
    out.ref_yaw       = xref_y;
    out.ref_yaw_vel   = vref_y; // 供位置环速度前馈
    out.ref_yaw_acc   = aref_y; // 供速度/电流环加速度前馈
    out.ref_pitch     = xref_p;
    out.ref_pitch_vel = vref_p;
    out.ref_pitch_acc = aref_p;

    plan_control_cmd_pub_->publish(out);

    // —— 前向积分内部虚拟状态，保持连续 —— //
    // 用“本拍实际用的 aref（= z(d)）”推进 1 步
    yaw_rate_hat_ += aref_y * T_;
    yaw_hat_ += yaw_rate_hat_ * T_;
    pitch_rate_hat_ += aref_p * T_;
    pitch_hat_ += pitch_rate_hat_ * T_;
}

} // namespace trajectory

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(trajectory::Planner)
