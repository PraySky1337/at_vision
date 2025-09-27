// planner_di.hpp
#pragma once
#include "target.hpp"
#include <Eigen/Dense>
#include <auto_aim_interfaces/msg/control_cmd.hpp>
#include <auto_aim_interfaces/msg/planned_control_cmd.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#if defined(__clang__)
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
#elif defined(__GNUC__)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wreturn-type-c-linkage"
#endif

#include "../../TinyMPC/src/tinympc/tiny_api.hpp"

#if defined(__clang__)
# pragma clang diagnostic pop
#elif defined(__GNUC__)
# pragma GCC diagnostic pop
#endif

#include <mutex>

namespace solver {

class Planner {
public:

    explicit Planner(rclcpp::Node* node);
    ~Planner();

    // 由外部回调/高频线程调用：输入 ControlCmd，返回一次规划结果
    // 由外部线程调用的情况下其参数热更新并不安全。
    PlanControlCmd process(const FireSolution& msg);

    // 可选：手动重置内部状态（滤波初值）
    void reset(double yaw = 0, double yaw_rate = 0, double pitch = 0, double pitch_rate = 0);

private:
    // ---- 依赖注入 Node ----
    rclcpp::Node::SharedPtr node_;
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

} // namespace solver
