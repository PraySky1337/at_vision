#pragma once

#include <Eigen/Dense>
#include <rm_interfaces/msg/target.hpp>
#include <vector>

namespace util {
enum State : uint8_t {
    XC    = 0,
    VX    = 1,
    YC    = 2,
    VY    = 3,
    ZC    = 4,
    VZ    = 5,
    YAW   = 6,
    V_YAW = 7,
    R     = 8,
    L     = 9,  // L = r2 - r1
    H     = 10, // z2 - z1
    STATE_MAX   = 11,
};

enum Measure : uint8_t {
    ARMOR_X   = 0,
    ARMOR_Y   = 1,
    ARMOR_Z   = 2,
    ARMOR_YAW = 3,
};

// 根据目标中心、yaw、半径等参数，计算各装甲板中心位置
std::vector<Eigen::Vector4d> getRoboArmorPose(
    const Eigen::Vector3d& target_center, double target_yaw, double radius, double l, double z0,
    double h, size_t armors_num);

// 从 Target 消息中直接计算装甲板中心位置
std::vector<Eigen::Vector4d> getRoboArmorPose(const rm_interfaces::msg::Target& target);

// 熵权法，输入：样本矩阵 X（行：样本，列：指标），输出：各指标权重
std::vector<double> entropy_weight(const std::vector<std::vector<double>>& X);

rm_interfaces::msg::Target state2target(const Eigen::MatrixXd& state);

Eigen::Vector3d state2armor_xyz(double const* x, int id, int armors_num);

double limit_rad(double angle);

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

} // namespace util
