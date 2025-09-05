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

    // 参考为常值
    Eigen::VectorXd ref_yaw   = Eigen::VectorXd::Constant(N_, ty);
    Eigen::VectorXd ref_pitch = Eigen::VectorXd::Constant(N_, tp);

    // 解 MPC（内部虚拟状态：yaw_hat_/yaw_rate_hat_）
    auto sy = ADMM_QP::for_mpc_with_smoothing(
        N_, T_, lambda_D_, amax_, yaw_hat_, yaw_rate_hat_, ref_yaw, rho_);
    auto sp = ADMM_QP::for_mpc_with_smoothing(
        N_, T_, lambda_D_, amax_, pitch_hat_, pitch_rate_hat_, ref_pitch, rho_);
    auto Ry     = sy.solve((warm_yaw_.size() == N_) ? &warm_yaw_ : nullptr);
    auto Rp     = sp.solve((warm_pitch_.size() == N_) ? &warm_pitch_ : nullptr);
    warm_yaw_   = Ry.z;
    warm_pitch_ = Rp.z;

    // —— 选拍：无明显时延 → 取第0拍；若有时延 d，用 z(d) —— //
    int d          = 0; // 有时延就改成 1 或 2
    double a_yaw   = std::clamp(Ry.z(d), -amax_, amax_);
    double a_pitch = std::clamp(Rp.z(d), -amax_, amax_);

    // 预测下一拍角度（作为 plan_* 给 PID）
    double yaw_rate_next   = yaw_rate_hat_ + a_yaw * T_;
    double pitch_rate_next = pitch_rate_hat_ + a_pitch * T_;
    double yaw_next        = yaw_hat_ + yaw_rate_hat_ * T_ + 0.5 * a_yaw * T_ * T_;
    double pitch_next      = pitch_hat_ + pitch_rate_hat_ * T_ + 0.5 * a_pitch * T_ * T_;

    // 发布（不含向量）
    PlannedControlCmd out;
    out.header.stamp = now();
    out.accel_yaw    = a_yaw;
    out.accel_pitch  = a_pitch;
    out.plan_yaw     = yaw_next; // 下一拍参考角
    out.plan_pitch   = pitch_next;

    // 简单开火逻辑（可按需改阈值/策略）
    double ang_thr = 0.01; // ≈0.57°（弧度）
    double vel_thr = 0.2;  // rad/s
    out.fire       = (std::abs(ty - yaw_next) < ang_thr && std::abs(yaw_rate_next) < vel_thr)
            && (std::abs(tp - pitch_next) < ang_thr && std::abs(pitch_rate_next) < vel_thr);

    plan_control_cmd_pub_->publish(out);

    // 前向积分虚拟状态（保持轨迹连续）
    yaw_rate_hat_   = yaw_rate_next;
    pitch_rate_hat_ = pitch_rate_next;
    yaw_hat_        = yaw_next;
    pitch_hat_      = pitch_next;
}

} // namespace trajectory

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(trajectory::Planner)
