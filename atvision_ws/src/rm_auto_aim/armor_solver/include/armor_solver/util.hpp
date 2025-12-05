#pragma once

#include <Eigen/Dense>
#include <rm_interfaces/msg/target.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace util {
enum State : uint8_t {
    XC,
    VX,
    YC,
    VY,
    Z0,    // 0,2 号装甲板的 Z
    VZ,
    YAW,   // 0 号装甲板的 YAW
    V_YAW, // 角速度
    R_0,
    R_1,
    H,     // Z1 - Z0
    STATE_MAX,
};

enum Measure : uint8_t {
    ARMOR_X,
    ARMOR_Y,
    ARMOR_Z,
    ARMOR_YAW,
    MEASURE_MAX
};

// 根据目标中心、yaw、半径等参数，计算各装甲板中心位置
std::vector<Eigen::Vector4d> get_robo_armor_poses(
    const Eigen::Vector3d& target_center, double target_yaw, double radius, double l, double z0,
    double h, size_t armors_num);

// 从 Target 消息中直接计算装甲板中心位置
std::vector<Eigen::Vector4d> get_robo_armor_poses(const rm_interfaces::msg::Target& target);

// 熵权法，输入：样本矩阵 X（行：样本，列：指标），输出：各指标权重
std::vector<double> entropy_weight(const std::vector<std::vector<double>>& X);

rm_interfaces::msg::Target state2target(const Eigen::MatrixXd& state);

Eigen::Vector3d state2armor_xyz(double const* x, int id, int armors_num);

inline Eigen::Vector3d pose_discard2position(const Eigen::Vector4d& pose) {
    return {pose.x(), pose.y(), pose.z()};
}

inline std::vector<Eigen::Vector3d>
    pose_discard2position(const std::vector<Eigen::Vector4d>& poses) {
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(poses.size());
    for (const auto& pose : poses) {
        positions.emplace_back(pose.x(), pose.y(), pose.z());
    }
    return positions;
}

inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) {
    double x = xyz.x();
    double y = xyz.y();
    double z = xyz.z();

    double distance = xyz.norm();
    double yaw      = std::atan2(y, x);
    double pitch    = std::atan2(z, std::sqrt(x * x + y * y));

    return Eigen::Vector3d(yaw, pitch, distance);
}

inline Eigen::Vector3d ypd2xyz(const Eigen::Vector3d& ypd) {
    double yaw      = ypd.x();
    double pitch    = ypd.y();
    double distance = ypd.z();

    double cp = std::cos(pitch);
    double x  = distance * cp * std::cos(yaw);
    double y  = distance * cp * std::sin(yaw);
    double z  = distance * std::sin(pitch);

    return Eigen::Vector3d(x, y, z);
}

inline double orientation2yaw(const geometry_msgs::msg::Quaternion& q_msg) {
    tf2::Quaternion q;
    tf2::fromMsg(q_msg, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

// 归一化到 (-PI, PI]
inline double normalize_rad(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a <= 0.0)
        a += 2.0 * M_PI;
    return a - M_PI;
}

// 计算最小角度差 to - from，结果在 (-PI, PI]
inline double shortest_rad(double from, double to) { return normalize_rad(to - from); }

inline double shortest_rad(double angle) { return normalize_rad(angle); }

// 根据上一帧角度 prev，使当前测量 raw 连续化
inline double unwrap_rad(double prev, double raw) {
    double d = shortest_rad(prev, raw);
    return prev + d;
}

// 归一化到 (-180, 180]
inline double normalize_deg(double a) {
    a = std::fmod(a + 180.0, 360.0);
    if (a <= 0.0)
        a += 360.0;
    return a - 180.0;
}

// 计算最小角度差 to - from，结果在 (-180, 180]
inline double shortest_deg(double from, double to) { return normalize_deg(to - from); }

inline double shortest_deg(double rad) { return normalize_deg(rad); }

// 连续化
inline double unwrap_deg(double prev, double raw) {
    double d = shortest_deg(prev, raw);
    return prev + d;
}

} // namespace util
