#pragma once
#include "admm_qp.hpp"
#include <Eigen/Dense>
#include <auto_aim_interfaces/msg/control_cmd.hpp>
#include <auto_aim_interfaces/msg/planned_control_cmd.hpp>
#include <rclcpp/rclcpp.hpp>

namespace trajectory {

class Planner : public rclcpp::Node {
public:
    explicit Planner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using Vec               = Eigen::VectorXd;
    using ControlCmd        = auto_aim_interfaces::msg::ControlCmd;
    using PlannedControlCmd = auto_aim_interfaces::msg::PlannedControlCmd;


    void onTarget(const ControlCmd::SharedPtr msg);
    void controlStep(); // timer 回调（固定周期）

    // 参数
    int N_;
    double T_, amax_, lambda_D_, rho_;

    // 目标（常值参考）
    std::atomic<double> tgt_yaw_{0.0}, tgt_pitch_{0.0};
    std::atomic<bool> has_tgt_{false};

    // 内部“虚拟状态”（仅用于规划起点，不是传感器）
    double yaw_hat_ = 0.0, pitch_hat_ = 0.0;           // \hatθ
    double yaw_rate_hat_ = 0.0, pitch_rate_hat_ = 0.0; // \hatω

    // warm start
    ADMM_QP::Vec warm_yaw_, warm_pitch_;

    // ROS
    rclcpp::Subscription<ControlCmd>::SharedPtr target_sub_;
    rclcpp::Publisher<PlannedControlCmd>::SharedPtr plan_control_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace trajectory
