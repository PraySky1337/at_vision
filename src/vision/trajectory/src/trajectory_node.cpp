#include "trajectory/trajectory_node.hpp"

#include <Eigen/Dense>
#include <auto_aim_interfaces/msg/target.hpp>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>

namespace {
double normalize_angle(double a)                 // 返回 -π ~ +π
{
    return std::atan2(std::sin(a), std::cos(a)); // atan2 保证周期归一
}
} // namespace
namespace trajectory {

/* ---------- 构造 ---------- */
TrajectoryNode::TrajectoryNode(
    const std::string& name, const std::string& ns, const rclcpp::NodeOptions& opt)
    : Node(name, ns, opt) {
    /* 读取参数 */
    bal_.v0 = declare_parameter("bullet_velocity", 25.0);
    bal_.k  = declare_parameter("resistance_coefficient", 0.0);
    bal_.g  = declare_parameter("gravity", 9.81);
    bal_.dt = declare_parameter("integration_step", 1e-3);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    target_sub_  = create_subscription<TargetMsg>(
        "armor_tracker/target", rclcpp::SensorDataQoS(),
        std::bind(&TrajectoryNode::target_call_back, this, std::placeholders::_1));
    control_cmd_pub_ = create_publisher<ControlCmd>("~/control_command", rclcpp::SensorDataQoS());
}

double TrajectoryNode::simulateY(double theta, const Target& tgt) {
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

double TrajectoryNode::solvePitch(const Target& tgt, double eps, int max_iter) {
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

void TrajectoryNode::target_call_back(const TargetMsg::SharedPtr msg) {
    /* ---------- 0. 跟踪状态检查 ---------- */
    if (!msg->tracking) {
        last_pitch_err_ = last_yaw_err_ = 0.0;
        return;
    }

    /* ---------- 1. 目标位姿/速度转到枪口系 ---------- */
    geometry_msgs::msg::PoseStamped odom_pose;
    geometry_msgs::msg::Vector3Stamped odom_vel;
    odom_pose.header        = msg->header;
    odom_pose.pose.position = msg->position;
    odom_vel.header         = msg->header;
    odom_vel.vector         = msg->velocity;

    const auto tf = tf_buffer_->lookupTransform("muzzle_link", "odom", rclcpp::Time(0));

    geometry_msgs::msg::PoseStamped muzzle_pose;
    geometry_msgs::msg::Vector3Stamped muzzle_vel;
    tf2::doTransform(odom_pose, muzzle_pose, tf);
    tf2::doTransform(odom_vel, muzzle_vel, tf);
    std_msgs::msg::Header header;
    header.frame_id  = "muzzle_link";
    header.stamp     = now();
    odom_pose.header = header;
    odom_vel.header  = header;

    Eigen::Vector3d p0(
        muzzle_pose.pose.position.x, muzzle_pose.pose.position.y, muzzle_pose.pose.position.z);
    Eigen::Vector3d v0(muzzle_vel.vector.x, muzzle_vel.vector.y, muzzle_vel.vector.z);

    /* ---------- 2. 线性预测装甲板中心 ---------- */
    const double dt    = bias_time_; // 秒
    Eigen::Vector3d pc = p0 + v0 * dt;

    /* ---------- 3. 生成装甲板中心坐标 ---------- */
    const std::size_t a_n = msg->armors_num; // 3/4
    const double r1       = msg->radius_1;
    const double r2       = msg->radius_2;
    const double dz       = msg->dz;
    const double yaw_c    = msg->yaw + msg->v_yaw * dt;

    std::vector<Eigen::Vector3d> armors;
    armors.reserve(a_n);

    switch (a_n) {
    case 3: {
        double r = 0.5 * (r1 + r2);
        for (std::size_t i = 0; i < 3; ++i) {
            double a = yaw_c + i * 2.0 * M_PI / 3.0;
            armors.emplace_back(pc.x() - r * cos(a), pc.y() - r * sin(a), pc.z());
        }
        break;
    }
    default: {
        bool use_r1 = true;
        for (std::size_t i = 0; i < 4; ++i) {
            double a = yaw_c + i * M_PI_2;
            double r = use_r1 ? r1 : r2;
            double z = pc.z() + (use_r1 ? 0.0 : dz);
            use_r1   = !use_r1;
            armors.emplace_back(pc.x() - r * cos(a), pc.y() - r * sin(a), z);
        }
    }
    }

    /* ---------- 4. 选择 yaw 误差最小的装甲板 ---------- */
    std::size_t best_idx = 0;
    double best_err      = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < armors.size(); ++i) {
        double yaw_to_armor = atan2(armors[i].y(), armors[i].x());
        double err          = fabs(normalize_angle(yaw_to_armor)); // 当前枪口 yaw = 0
        if (err < best_err) {
            best_err = err;
            best_idx = i;
        }
    }

    Eigen::Vector3d best_pos = armors[best_idx];
    double best_yaw          = atan2(best_pos.y(), best_pos.x());

    /* ---------- 5. yaw_err ---------- */
    double yaw_err = normalize_angle(best_yaw);

    /* ---------- 6. pitch_err ---------- */
    double s_xy   = hypot(best_pos.x(), best_pos.y());
    double z_diff = best_pos.z();
    Target tgt{.d = s_xy, .h = z_diff};
    double ideal_pitch = -solvePitch(tgt); // rad
    double pitch_err   = normalize_angle(ideal_pitch);

    /* ---------- 7. 保存 / 发布 ---------- */
    last_pitch_err_ = pitch_err;
    last_yaw_err_   = yaw_err;

    ControlCmd control_cmd;
    control_cmd.is_large_armor = msg->id == "1";
    control_cmd.header         = header;
    control_cmd.tracking       = msg->tracking;
    control_cmd.pitch_error    = last_pitch_err_;
    control_cmd.yaw_error      = last_yaw_err_;
    control_cmd_pub_->publish(control_cmd);
}
} // namespace trajectory
