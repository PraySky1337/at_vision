#pragma once
#include <auto_aim_interfaces/msg/control_cmd.hpp>
#include <auto_aim_interfaces/msg/detail/control_cmd__struct.hpp>
#include <auto_aim_interfaces/msg/target.hpp>

#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace trajectory {

class TrajectoryNode : public rclcpp::Node {
public:
    explicit TrajectoryNode(
        const std::string& name = "trajectory_node", const std::string& ns = "",
        const rclcpp::NodeOptions& opt = rclcpp::NodeOptions{});

private:
    static constexpr const uint8_t OUTPOST_ARMOR_NUM = 3;
    static constexpr const uint8_t NORMAL_ARMOR_NUM  = 4;
    struct Target {
        double d ; 
        double h ;
    };

    struct Ballistics {
        double v0 = 25.0; // 初速  (m/s)
        double k  = 0.0;  // 空气阻力系数 ρCdA/(2m)  (1/m)
        double g  = 9.81; // 重力  (m/s²)
        double dt = 1e-3; // 数值积分步长 (s) (k>0 时用)
    } bal_;

    double bias_time_;
    /// 无/有阻力下，给定 theta 计算落点 y(d)
    double simulateY(double theta, const Target& tgt);

    /// 二分法解俯仰角，使 simulateY(theta) == tgt.h
    double solvePitch(const Target& tgt, double eps = 1e-4, int max_iter = 20);

    using TargetMsg  = auto_aim_interfaces::msg::Target;
    using ControlCmd = auto_aim_interfaces::msg::ControlCmd;
    void target_call_back(const TargetMsg::SharedPtr target_msg);

    rclcpp::Subscription<TargetMsg>::SharedPtr target_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<ControlCmd>::SharedPtr control_cmd_pub_;

    double last_yaw_err_;
    double last_pitch_err_;
};

} // namespace trajectory
