#pragma once
#include <auto_aim_interfaces/msg/control_cmd.hpp>
#include <auto_aim_interfaces/msg/target.hpp>

#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace trajectory {

class TrajectoryDriver : public rclcpp::Node {
public:
    explicit TrajectoryDriver(const rclcpp::NodeOptions& options);

private:
    static constexpr const uint8_t OUTPOST_ARMOR_NUM = 3;
    static constexpr const uint8_t NORMAL_ARMOR_NUM  = 4;
    static constexpr const double ARROW_LEN          = 1.0;
    struct Target {
        double d;
        double h;
    };

    struct Ballistics {
        double v0 = 21.0; // 初速  (m/s)
        double k  = 0.0;  // 空气阻力系数 ρCdA/(2m)  (1/m)
        double g  = 9.81; // 重力  (m/s²)
        double dt = 1e-3; // 数值积分步长 (s) (k>0 时用)
    } bal_;

    double bias_time_;
    /// 无/有阻力下，给定 theta 计算落点 y(d)
    double simulateY(double theta, const Target& tgt);

    /// 二分法解俯仰角，使 simulateY(theta) == tgt.h
    double solvePitch(const Target& tgt, double eps = 1e-4, int max_iter = 50);

    using TargetMsg  = auto_aim_interfaces::msg::Target;
    using ControlCmd = auto_aim_interfaces::msg::ControlCmd;
    void target_call_back(const TargetMsg::SharedPtr target_msg);

    rclcpp::Subscription<TargetMsg>::SharedPtr target_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<ControlCmd>::SharedPtr control_cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr aim_marker_pub_;
    visualization_msgs::msg::MarkerArray mk_array_; // 蓝箭：当前方向
    visualization_msgs::msg::Marker mk_current_;    // 蓝箭：当前方向
    visualization_msgs::msg::Marker mk_target_;     // 红箭：期望方向
};

} // namespace trajectory
