#pragma once

#include "tiny_api.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/plan_gimbal_cmd.hpp>

namespace rm_gimbal {

struct Planner {
    explicit Planner(rclcpp::Node::WeakPtr node);
    ~Planner();
    rm_interfaces::msg::PlanGimbalCmd process(const rm_interfaces::msg::GimbalCmd& msg);
    void reset(double yaw = 0, double yaw_rate = 0, double pitch = 0, double pitch_rate = 0);

private:
    // ---- 依赖注入 Node ----
    rclcpp::Node::WeakPtr node_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    rcl_interfaces::msg::SetParametersResult onParam(const std::vector<rclcpp::Parameter>& ps);

    // ---- 参数（与原版一致）----
    int N_;                  // 时域长度
    double T_;               // 采样周期
    double amax_;            // 加速度上限
    double dt_min_, dt_max_; // dt 限幅
    double dt_ema_alpha_;    // dt EMA 平滑
    // 权重
    double q_yaw_pos_, q_yaw_vel_, r_yaw_acc_;
    double q_pitch_pos_, q_pitch_vel_, r_pitch_acc_;

    // 内部状态（估计/预测）
    double yaw_hat_ = 0.0, yaw_rate_hat_ = 0.0;
    double pitch_hat_ = 0.0, pitch_rate_hat_ = 0.0;

    std::chrono::steady_clock::time_point last_now_{};
    bool have_last_now_{false};

    // 可配：是否用内部时钟估计 dt（默认 true）
    double last_dt_{0.0};

    std::mutex param_mtx_;

    // TinyMPC 求解器
    TinySolver* yaw_solver_   = nullptr;
    TinySolver* pitch_solver_ = nullptr;

    // 构建/释放
    static void destroySolver(TinySolver*& s);
    static void buildOneSolver(
        TinySolver*& solver, int N, double dt, double amax, double q_pos, double q_vel,
        double r_acc);
    void rebuildSolvers(double dt);
};

} // namespace rm_gimbal