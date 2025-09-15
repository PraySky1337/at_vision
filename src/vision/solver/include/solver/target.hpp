#pragma once

#include <auto_aim_interfaces/msg/planned_control_cmd.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/time.hpp>

namespace solver {

enum ArmorType { SMALL, LARGE };

struct Target {
    static constexpr double SMALL_ARMOR_WIDTH = 0.135; // m
    static constexpr double LARGE_ARMOR_WIDTH = 0.23;  // m
    rclcpp::Time stamp;
    bool tracking;
    std::string id;
    int armors_num;                                    // 装甲板数量
    geometry_msgs::msg::Point position;                // odom坐标系下的三维位置
    geometry_msgs::msg::Vector3 velocity;              // odom坐标系下的三维速度
    double yaw;
    double v_yaw;
    double radius_1;
    double radius_2;
    double dz;
    void fromMsg(const auto_aim_interfaces::msg::Target& msg) {
        stamp      = msg.header.stamp;
        tracking   = msg.tracking;
        id         = msg.id;
        armors_num = msg.armors_num;
        position   = msg.position;
        velocity   = msg.velocity;
        yaw        = msg.yaw;
        v_yaw      = msg.v_yaw;
        radius_1   = msg.radius_1;
        radius_2   = msg.radius_2;
        dz         = msg.dz;
    }
    ArmorType armor_type() {
        if (id == "1") {
            return LARGE;
        } else {
            return SMALL;
        }
    }
};

struct PlanControlCmd {
    float target_yaw;
    float target_pitch;
    float target_yaw_vel;
    float target_pitch_vel;
    float target_yaw_acc;
    float target_pitch_acc;
    auto_aim_interfaces::msg::PlannedControlCmd toMsg() {
        auto_aim_interfaces::msg::PlannedControlCmd msg;
        msg.ref_yaw       = target_yaw;
        msg.ref_pitch     = target_pitch;
        msg.ref_yaw_vel   = target_yaw_vel;
        msg.ref_pitch_vel = target_pitch_vel;
        msg.ref_yaw_acc   = target_yaw_acc;
        msg.ref_pitch_acc = target_pitch_acc;
        return msg;
    }
    void fromMsg(const auto_aim_interfaces::msg::PlannedControlCmd& msg) {
        target_yaw       = msg.ref_yaw;
        target_pitch     = msg.ref_pitch;
        target_yaw_vel   = msg.ref_yaw_vel;
        target_pitch_vel = msg.ref_pitch_vel;
        target_yaw_acc   = msg.ref_yaw_acc;
        target_pitch_acc = msg.ref_pitch_acc;
    }
};

struct FireSolution {
    double target_pitch;
    double target_yaw;
    double timeoflight;
    bool ok;
};

} // namespace solver