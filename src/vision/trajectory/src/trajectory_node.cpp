#include "trajectory/trajectory_node.hpp"

#include <Eigen/Dense>
#include <auto_aim_interfaces/msg/target.hpp>
#include <cmath>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>

namespace trajectory {

/* ---------- 构造 ---------- */
TrajectoryDriver::TrajectoryDriver(const rclcpp::NodeOptions& options)
    : rclcpp::Node("trajectory", options) {
    /* 读取参数 */
    bal_.v0    = declare_parameter("bullet_velocity", 21.0);
    bal_.k     = declare_parameter("resistance_coefficient", 0.0);
    bal_.g     = declare_parameter("gravity", 9.83);
    bal_.dt    = declare_parameter("integration_step", 1e-3);
    bias_time_ = declare_parameter("bias_time", 0.01);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    target_sub_  = create_subscription<TargetMsg>(
        "tracker/target", rclcpp::SensorDataQoS(),
        std::bind(&TrajectoryDriver::target_call_back, this, std::placeholders::_1));
    control_cmd_pub_ =
        create_publisher<ControlCmd>("trajectory/control_command", rclcpp::SensorDataQoS());
    aim_marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("trajectory/aim_line", 10);

    auto init_arrow = [&](int id, float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.ns                 = "aim_line";
        m.id                 = id;
        m.type               = visualization_msgs::msg::Marker::ARROW;
        m.action             = visualization_msgs::msg::Marker::ADD;
        m.lifetime           = rclcpp::Duration::from_seconds(0.1);
        m.scale.x            = 0.015;                                          // shaft
        m.scale.y            = 0.03;                                           // head 直径
        m.scale.z            = 0.06;                                           // head 长度
        m.color.r            = r;
        m.color.g            = g;
        m.color.b            = b;
        m.color.a            = 1.0f;
        m.pose.orientation.w = 1.0;                                            // 单位四元数
        m.header.frame_id    = "muzzle_link";                                  // 坐标系固定
        return m;
    };

    visualization_msgs::msg::Marker current = init_arrow(0, 0.1f, 0.2f, 1.0f); // 蓝
    visualization_msgs::msg::Marker target  = init_arrow(1, 1.0f, 0.1f, 0.1f); // 红
    mk_array_.markers                       = {current, target};
}

double TrajectoryDriver::simulateY(double theta, const Target& tgt) {
    // 无阻力解析解
    if (bal_.k == 0.0) {
        double vx = bal_.v0 * std::cos(theta);
        double vy = bal_.v0 * std::sin(theta);
        double t  = tgt.d / vx;
        return vy * t - 0.5 * bal_.g * t * t;
    }

    // 有阻力 Euler 积分
    double x = 0.0, y = 0.0;
    double vx = bal_.v0 * std::cos(theta);
    double vy = bal_.v0 * std::sin(theta);

    while (x < tgt.d) {
        double v  = std::hypot(vx, vy);
        double ax = -bal_.k * v * vx;
        double ay = -bal_.g - bal_.k * v * vy;
        x += vx * bal_.dt;
        y += vy * bal_.dt;
        vx += ax * bal_.dt;
        vy += ay * bal_.dt;
    }
    return y;
}

double TrajectoryDriver::solvePitch(const Target& tgt, double eps, int max_iter) {
    double lo = -M_PI / 8, hi = M_PI / 3;
    double y_lo = simulateY(lo, tgt) - tgt.h;
    double y_hi = simulateY(hi, tgt) - tgt.h;
    if (y_lo * y_hi > 0) {
        return 0.0;
    }

    for (int i = 0; i < max_iter; ++i) {
        double mid   = 0.5 * (lo + hi);
        double y_mid = simulateY(mid, tgt) - tgt.h;
        if (std::abs(y_mid) < eps)
            return mid;
        if (y_mid * y_lo < 0)
            hi = mid;
        else {
            lo   = mid;
            y_lo = y_mid;
        }
    }
    return 0.5 * (lo + hi);
}

void TrajectoryDriver::target_call_back(const TargetMsg::SharedPtr msg) {
    if (!msg->tracking) {
        return;
    }
    const double xc = msg->position.x, yc = msg->position.y, zc = msg->position.z;
    const double vx = msg->velocity.x, vy = msg->velocity.y, vz = msg->velocity.z;
    const double yaw_c = msg->yaw;
    const double r1    = msg->radius_1;
    const double r2    = msg->radius_2;
    const double dz    = msg->dz;
    std::size_t a_n    = msg->armors_num;
    std::string id     = msg->id;
    std::vector<Eigen::Vector3d> armors;
    armors.reserve(a_n);
    bool use_r1 = true;
    for (std::size_t i = 0; i < a_n; ++i) {
        const double a = yaw_c + i * 2.0 * M_PI / a_n; // 3 块：120° 间隔；4 块：90° 间隔
        const double r = (a_n == 4) ? (use_r1 ? r1 : r2) : r1;
        const double z = (a_n == 4) ? (zc + (use_r1 ? 0.0 : dz)) : zc;
        use_r1         = !use_r1;                      // 4 装甲交替半径/高度

        double x = xc - r * std::cos(a);
        double y = yc - r * std::sin(a);
        armors.emplace_back(x, y, z);
    }

    if (armors.empty())
        return;                                        // 双保险

    std::size_t best_idx = 0;
    double best_dist     = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < armors.size(); ++i) {
        const auto& p = armors[i];
        double dist   = std::hypot(p.x(), std::hypot(p.y(), p.z())); // 3D 距离
        if (dist < best_dist) {
            best_dist = dist;
            best_idx  = i;
        }
    }
    Eigen::Vector3d best_pos = armors[best_idx];
    best_pos += bias_time_ * Eigen::Vector3d{vx, vy, vz};
    // geometry_msgs::msg::PointStamped best_point;
    // best_point.point.set__x(best_pos.x()).set__y(best_pos.y()).set__z(best_pos.z());
    // tf_buffer_.transform("muzzle_link","");
    double best_yaw   = std::atan2(best_pos.y(), best_pos.x());
    double horiz_dist = std::hypot(best_pos.x(), best_pos.y());
    double best_pitch = std::atan2(-best_pos.z(), horiz_dist);

    ControlCmd control_cmd;
    control_cmd.header         = msg->header;
    control_cmd.is_large_armor = msg->id == "1";
    control_cmd.target_pitch   = best_pitch;
    control_cmd.target_yaw     = best_yaw;
    control_cmd.tracking       = msg->tracking;
    control_cmd_pub_->publish(control_cmd);
}

} // namespace trajectory

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory::TrajectoryDriver)
