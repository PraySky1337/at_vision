#include "solver/trajectory_node.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <auto_aim_interfaces/msg/target.hpp>
#include <cmath>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rcl/event.h>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace solver {

/* ---------- 构造 ---------- */
Trajectory::Trajectory(const rclcpp::NodeOptions& options)
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
        std::bind(&Trajectory::target_call_back, this, std::placeholders::_1));
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

double Trajectory::simulateY(double theta, double d, double h) {
    // 无阻力解析解
    if (bal_.k == 0.0) {
        double vx = bal_.v0 * std::cos(theta);
        double vy = bal_.v0 * std::sin(theta);
        double t  = d / vx;
        return vy * t - 0.5 * bal_.g * t * t;
    }

    // 有阻力 Euler 积分
    double x = 0.0, y = 0.0;
    double vx = bal_.v0 * std::cos(theta);
    double vy = bal_.v0 * std::sin(theta);

    while (x < d) {
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

double Trajectory::solvePitch(double d, double h, double eps, int max_iter) {
    double lo = 0.0, hi = M_PI / 3;
    double y_lo = simulateY(lo, d, h) - h;
    double y_hi = simulateY(hi, d, h) - h;
    if (y_lo * y_hi > 0) {
        return 0.0;
    }

    for (int i = 0; i < max_iter; ++i) {
        double mid   = 0.5 * (lo + hi);
        double y_mid = simulateY(mid, d, h) - h;
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

void Trajectory::target_call_back(const TargetMsg::SharedPtr msg) {
    if (!msg->tracking) return;

    // 1) odom→muzzle 变换（仅用到平移，因结果发布在 odom）
    Eigen::Isometry3d T_odom_muzzle;
    try {
        auto tf_msg = tf_buffer_->lookupTransform(
            "odom", "muzzle_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
        t_odom_muzzle_full_ = tf2::transformToEigen(tf_msg.transform);
        T_odom_muzzle       = t_odom_muzzle_full_;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get static tf: %s", ex.what());
        return;
    }

    // 2) 生成装甲板候选点（odom）
    const double xc = msg->position.x, yc = msg->position.y, zc = msg->position.z;
    const double vx = msg->velocity.x, vy = msg->velocity.y, vz = msg->velocity.z;
    const double yaw_c = msg->yaw;
    const double r1 = msg->radius_1, r2 = msg->radius_2, dz = msg->dz;
    const std::size_t a_n = msg->armors_num;

    std::vector<Eigen::Vector3d> armors;
    armors.reserve(a_n);
    bool use_r1 = true;
    for (std::size_t i = 0; i < a_n; ++i) {
        const double a = yaw_c + i * 2.0 * M_PI / a_n;
        const double r = (a_n == 4) ? (use_r1 ? r1 : r2) : r1;
        const double z = (a_n == 4) ? (zc + (use_r1 ? 0.0 : dz)) : zc;
        use_r1         = !use_r1;
        double x       = xc - r * std::cos(a);
        double y       = yc - r * std::sin(a);
        armors.emplace_back(x, y, z);
    }
    if (armors.empty()) return;

    // 3) 选最近
    std::size_t best_idx = 0;
    double best_dist = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < armors.size(); ++i) {
        const auto& p = armors[i];
        double dist = std::hypot(p.x(), std::hypot(p.y(), p.z()));
        if (dist < best_dist) { best_dist = dist; best_idx = i; }
    }

    // 4) 目标点时间前推（仍在 odom）
    Eigen::Vector3d best_pos = armors[best_idx] + bias_time_ * Eigen::Vector3d{vx, vy, vz};

    // 5) 相对向量（odom）：只做减法即可（两点都在 odom）
    const Eigen::Vector3d muzzle_in_odom = T_odom_muzzle.translation();
    const Eigen::Vector3d rel = best_pos - muzzle_in_odom;

    // 6) 弹道解算（基于 odom 的水平距离 d 与高度差 h）
    const double d = std::hypot(rel.x(), rel.y());
    const double h = rel.z();
    const double theta = solvePitch(d, h);   // 相对于水平面的仰角（rad）

    // 7) 角度（都在 odom 系）
    //   - 你的旧实现等价于“向下为正”，沿用此约定：pitch = -theta
    const double pitch = -theta;
    const double yaw   = std::atan2(rel.y(), rel.x());

    // 8) 发布（odom 系）
    ControlCmd control_cmd;
    control_cmd.header       = msg->header;
    control_cmd.target_pitch = pitch;
    control_cmd.target_yaw   = yaw;
    control_cmd_pub_->publish(control_cmd);

    // 9) 可视化：仍按你原先逻辑在 odom 下画期望箭头
    this->publish_marker(control_cmd, best_pos);
}

void Trajectory::publish_marker(const ControlCmd& msg, const Eigen::Vector3d& best_pos) {
    // 枪口中心在odom下的位置
    Eigen::Vector3d muzzle_in_odom = t_odom_muzzle_full_.translation();

    // 目标点在odom
    Eigen::Vector3d target_in_odom =
        best_pos;

    // 目标方向
    Eigen::Vector3d aim_direction = (target_in_odom - muzzle_in_odom).normalized();

    // 枪口正前方
    Eigen::Vector3d muzzle_axis_in_odom = t_odom_muzzle_full_.linear() * Eigen::Vector3d::UnitX();

    double line_length = ARROW_LEN; // 可调整

    // 1. 红箭——期望方向
    mk_target_.header.frame_id = "odom";
    mk_target_.header.stamp    = msg.header.stamp;
    mk_target_.ns              = "trajectory";
    mk_target_.id              = 0;
    mk_target_.type            = visualization_msgs::msg::Marker::ARROW;
    mk_target_.action          = visualization_msgs::msg::Marker::ADD;
    mk_target_.scale.x         = 0.03;
    mk_target_.scale.y         = 0.06;
    mk_target_.scale.z         = 0.08;
    mk_target_.color.r         = 1.0;
    mk_target_.color.g         = 0.0;
    mk_target_.color.b         = 0.0;
    mk_target_.color.a         = 1.0;

    geometry_msgs::msg::Point p0, p1;
    p0.x              = muzzle_in_odom.x();
    p0.y              = muzzle_in_odom.y();
    p0.z              = muzzle_in_odom.z();
    p1.x              = p0.x + aim_direction.x() * line_length;
    p1.y              = p0.y + aim_direction.y() * line_length;
    p1.z              = p0.z + aim_direction.z() * line_length;
    mk_target_.points = {p0, p1};

    // 2. 蓝箭——当前方向
    mk_current_             = mk_target_; // 复制基本属性
    mk_current_.id          = 1;
    mk_current_.color.r     = 0.0;
    mk_current_.color.g     = 0.0;
    mk_current_.color.b     = 1.0;
    mk_current_.points[1].x = p0.x + muzzle_axis_in_odom.x() * line_length;
    mk_current_.points[1].y = p0.y + muzzle_axis_in_odom.y() * line_length;
    mk_current_.points[1].z = p0.z + muzzle_axis_in_odom.z() * line_length;
    mk_array_.markers       = {mk_target_, mk_current_};
    aim_marker_pub_->publish(mk_array_);
}

Trajectory::~Trajectory() { rclcpp::shutdown(); }

} // namespace trajectory

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(solver::Trajectory)
