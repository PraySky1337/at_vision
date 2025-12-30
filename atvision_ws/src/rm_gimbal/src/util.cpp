#include "rm_gimbal/util.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace rm_gimbal {
namespace util {

std::vector<Eigen::Vector4d> get_robo_armor_poses(
    const Eigen::Vector3d& target_pos, const double yaw, const double radius0, const double radius1,
    const double z1, const double z2, const size_t armors_num) {
    std::vector<Eigen::Vector4d> poses(armors_num);
    const double angle_step = 2.0 * M_PI / static_cast<double>(armors_num);
    const double cx         = target_pos.x();
    const double cy         = target_pos.y();
    const double z0         = target_pos.z();

    for (size_t i = 0; i < armors_num; ++i) {
        const bool another_pair = (armors_num == 4) && (i == 1 || i == 3);
        const double r          = another_pair ? radius1 : radius0;
        const double angle      = normalize_rad(yaw + angle_step * static_cast<double>(i));
        poses[i].x()            = cx - r * std::cos(angle);
        poses[i].y()            = cy - r * std::sin(angle);

        if (armors_num == 3) {
            // Outpost: three plates stacked vertically
            poses[i].z() = (i == 0) ? z0 : (i == 1) ? z1 : z2;
        } else if (armors_num == 4) {
            poses[i].z() = another_pair ? z1 : z0;
        } else {
            throw std::runtime_error("invalid armors num");
        }
        poses[i].w() = angle;
    }

    return poses;
}

std::vector<Eigen::Vector4d> get_outpost_armor_poses(
    const Eigen::Vector3d& target_pos, double yaw, double radius0, double z0, double z1,
    double z2) {
    std::vector<Eigen::Vector4d> poses(3);
    constexpr double angle_step = 2.0 * M_PI / 3;

    const double cx = target_pos.x();
    const double cy = target_pos.y();

    for (size_t i = 0; i < 3; ++i) {
        const double angle = normalize_rad(yaw + angle_step * static_cast<double>(i));

        poses[i].x() = cx - radius0 * std::cos(angle);
        poses[i].y() = cy - radius0 * std::sin(angle);
        poses[i].z() = (i == 0) ? z0 : (i == 1) ? z1 : z2;
        poses[i].w() = angle;
    }

    return poses;
}

std::vector<Eigen::Vector4d> get_robo_armor_poses(const rm_interfaces::msg::Target& target) {
    Eigen::Vector3d target_center(target.position.x, target.position.y, target.position.z);
    return get_robo_armor_poses(
        target_center, target.yaw, target.radius0, target.radius1, target.z1, target.z2,
        target.armors_num);
}

} // namespace util
} // namespace rm_gimbal
