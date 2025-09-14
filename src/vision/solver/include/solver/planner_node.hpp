#pragma once
#include <Eigen/Dense>
#include <auto_aim_interfaces/msg/control_cmd.hpp>
#include <auto_aim_interfaces/msg/planned_control_cmd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#if defined(__clang__)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
#elif defined(__GNUC__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wreturn-type-c-linkage"
#endif

#include "../../TinyMPC/src/tinympc/tiny_api.hpp"

#if defined(__clang__)
  #pragma clang diagnostic pop
#elif defined(__GNUC__)
  #pragma GCC diagnostic pop
#endif

#include <mutex>

namespace solver {

class Planner : public rclcpp::Node {
public:
    explicit Planner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~Planner();

private:
    using ControlCmd        = auto_aim_interfaces::msg::ControlCmd;
    using PlannedControlCmd = auto_aim_interfaces::msg::PlannedControlCmd;

    // 回调
    void onTarget(const ControlCmd::ConstSharedPtr msg);
    rcl_interfaces::msg::SetParametersResult onParam(const std::vector<rclcpp::Parameter>& ps);

    // ---- 参数（仅 TinyMPC 相关）----
    int    N_;                 // 时域长度
    double T_;                 // 采样周期
    double amax_;              // 加速度上限
    double dt_min_, dt_max_;   // dt 限幅
    double dt_ema_alpha_;      // dt EMA 平滑
    // 权重
    double q_yaw_pos_,  q_yaw_vel_,  r_yaw_acc_;
    double q_pitch_pos_,q_pitch_vel_,r_pitch_acc_;

    // 内部状态（估计/预测）
    double yaw_hat_ = 0.0, yaw_rate_hat_ = 0.0;
    double pitch_hat_ = 0.0, pitch_rate_hat_ = 0.0;

    // 时间基
    bool have_last_stamp_ = false;
    rclcpp::Time last_stamp_;
    double last_dt_;

    // ROS
    rclcpp::Subscription<ControlCmd>::SharedPtr target_sub_;
    rclcpp::Publisher<PlannedControlCmd>::SharedPtr plan_control_cmd_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    std::mutex param_mtx_;

    // TinyMPC 求解器
    TinySolver* yaw_solver_   = nullptr;
    TinySolver* pitch_solver_ = nullptr;

    // 构建/释放
    static void destroySolver(TinySolver*& s);
    static void buildOneSolver(
        TinySolver*& solver, int N, double dt, double amax,
        double q_pos, double q_vel, double r_acc);
    void rebuildSolvers(double dt);
};

} // namespace solver
