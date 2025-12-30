#include "rm_gimbal/trajectory_generator.hpp"
#include "rm_gimbal/util.hpp"

#include <cmath>

namespace rm_gimbal {

// Trajectory methods
Eigen::MatrixXd Trajectory::getYawRefMatrix() const {
    Eigen::MatrixXd ref(2, horizon);  // [yaw, yaw_vel]
    for (int i = 0; i < horizon; ++i) {
        ref(0, i) = points[i].yaw;
        // 计算yaw速度（有限差分）
        if (i < horizon - 1) {
            ref(1, i) = (points[i + 1].yaw - points[i].yaw) / dt;
        } else {
            ref(1, i) = ref(1, i - 1);  // 最后一个点使用前一个速度
        }
    }
    return ref;
}

Eigen::MatrixXd Trajectory::getPitchRefMatrix() const {
    Eigen::MatrixXd ref(2, horizon);  // [pitch, pitch_vel]
    for (int i = 0; i < horizon; ++i) {
        ref(0, i) = points[i].pitch;
        // 计算pitch速度（有限差分）
        if (i < horizon - 1) {
            ref(1, i) = (points[i + 1].pitch - points[i].pitch) / dt;
        } else {
            ref(1, i) = ref(1, i - 1);
        }
    }
    return ref;
}

// TrajectoryGenerator methods
TrajectoryGenerator::TrajectoryGenerator(const GeneratorParams& params)
    : params_(params) {}

void TrajectoryGenerator::updateParams(const GeneratorParams& params) {
    params_ = params;
}

Trajectory TrajectoryGenerator::generateShootingTrajectory(
    const rm_interfaces::msg::Target& target,
    const Eigen::Vector3d& muzzle_position,
    double bullet_speed,
    const std::unique_ptr<fyt::TrajectoryCompensator>& trajectory_compensator) {

    Trajectory trajectory;
    trajectory.dt = params_.dt;
    trajectory.horizon = params_.horizon();
    trajectory.points.reserve(params_.horizon());

    int prev_armor_id = -1;

    // 中心对齐轨迹：从 -half_horizon * dt 到 +half_horizon * dt
    // 当前时刻对应 half_horizon 位置
    const int half_horizon = params_.half_horizon;

    for (int step = 0; step < params_.horizon(); ++step) {
        // 相对于当前时刻的时间偏移
        // step=0 对应 t = -half_horizon * dt（过去）
        // step=half_horizon 对应 t = 0（当前）
        // step=2*half_horizon-1 对应 t = (half_horizon-1) * dt（未来）
        double t = (step - half_horizon) * params_.dt;

        // 1. 预测t时刻的目标状态
        PredictedState state = predictTargetState(target, t);

        // 2. 获取装甲板位置
        auto armor_positions = getArmorPositions(target, state);

        // 3. 选择最佳装甲板
        int armor_id = selectBestArmor(
            armor_positions, muzzle_position, state.yaw, target.v_yaw, target.armors_num);

        // 4. 计算子弹飞行时间
        Eigen::Vector3d armor_pos(
            armor_positions[armor_id].x(),
            armor_positions[armor_id].y(),
            armor_positions[armor_id].z());
        double t_fly = calculateFlyingTime(
            armor_pos, muzzle_position, bullet_speed, trajectory_compensator);

        // 5. 重新预测（考虑飞行时间，提前t_fly）
        PredictedState shooting_state = predictTargetState(target, t + t_fly);
        auto shooting_armor_positions = getArmorPositions(target, shooting_state);

        // 使用同一个装甲板ID计算射击位置
        Eigen::Vector3d shooting_armor_pos(
            shooting_armor_positions[armor_id].x(),
            shooting_armor_positions[armor_id].y(),
            shooting_armor_positions[armor_id].z());

        // 6. 计算yaw和pitch（相对于枪口）
        double yaw, pitch;
        calculateYawPitch(shooting_armor_pos, muzzle_position, yaw, pitch);

        // 7. 检测装甲板切换
        bool is_switching = (prev_armor_id >= 0 && armor_id != prev_armor_id);

        // 8. 构建轨迹点
        TrajectoryPoint point;
        point.time = t;  // 注意：这里是相对时间，可以是负数
        point.yaw = yaw;
        point.pitch = pitch;
        point.armor_id = armor_id;
        point.is_switching = is_switching;

        trajectory.points.push_back(point);
        prev_armor_id = armor_id;
    }

    return trajectory;
}

TrajectoryGenerator::PredictedState TrajectoryGenerator::predictTargetState(
    const rm_interfaces::msg::Target& target,
    double dt) const {

    PredictedState state;

    // 匀速运动预测
    state.position.x() = target.position.x + dt * target.velocity.x;
    state.position.y() = target.position.y + dt * target.velocity.y;
    state.position.z() = target.position.z + dt * target.velocity.z;
    state.yaw = target.yaw + dt * target.v_yaw;

    return state;
}

std::vector<Eigen::Vector4d> TrajectoryGenerator::getArmorPositions(
    const rm_interfaces::msg::Target& target,
    const PredictedState& state) const {

    if (target.armors_num == 4) {
        return util::get_robo_armor_poses(
            state.position, state.yaw,
            target.radius0, target.radius1,
            target.z1, target.z2,
            target.armors_num);
    } else if (target.armors_num == 3) {
        return util::get_outpost_armor_poses(
            state.position, state.yaw,
            target.radius0,
            target.position.z, target.z1, target.z2);
    }

    // 默认返回空
    return {};
}

int TrajectoryGenerator::selectBestArmor(
    const std::vector<Eigen::Vector4d>& armor_positions,
    const Eigen::Vector3d& muzzle_position,
    double target_yaw,
    double target_v_yaw,
    size_t armors_num) const {

    // 计算目标中心相对于枪口的方向
    Eigen::Vector3d target_center = Eigen::Vector3d::Zero();
    for (const auto& pos : armor_positions) {
        target_center += pos.head<3>();
    }
    target_center /= static_cast<double>(armor_positions.size());
    target_center -= muzzle_position;

    // Angle between the car's center and the X-axis
    double alpha = std::atan2(target_center.y(), target_center.x());
    // Angle between the front of observed armor and the X-axis
    double beta = target_yaw;

    // 旋转矩阵
    Eigen::Matrix2d R_odom2center;
    Eigen::Matrix2d R_odom2armor;
    R_odom2center << std::cos(alpha), std::sin(alpha),
                    -std::sin(alpha), std::cos(alpha);
    R_odom2armor << std::cos(beta), std::sin(beta),
                   -std::sin(beta), std::cos(beta);

    Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;
    double decision_angle = std::atan2(R_center2armor(1, 0), R_center2armor(0, 0));

    // 根据角速度方向确定提前量
    double theta = (target_v_yaw > 0 ? params_.side_angle : -params_.side_angle) / 180.0 * M_PI;

    // 避免角速度过小时频繁切换
    if (std::abs(target_v_yaw) < params_.min_switching_v_yaw) {
        theta = 0;
    }

    const double step = 2 * M_PI / static_cast<double>(armors_num);
    double temp_angle = decision_angle + step / 2 - theta;
    temp_angle = std::fmod(temp_angle + 2 * M_PI, 2 * M_PI);  // wrap to [0, 2π)
    int selected_id = static_cast<int>(temp_angle / step);

    return selected_id;
}

double TrajectoryGenerator::calculateFlyingTime(
    const Eigen::Vector3d& armor_position,
    const Eigen::Vector3d& muzzle_position,
    double bullet_speed,
    const std::unique_ptr<fyt::TrajectoryCompensator>& trajectory_compensator) const {

    Eigen::Vector3d relative_pos = armor_position - muzzle_position;

    if (trajectory_compensator) {
        return trajectory_compensator->getFlyingTime(relative_pos);
    }

    // 简单估算：距离 / 速度
    double distance = relative_pos.norm();
    return distance / bullet_speed;
}

void TrajectoryGenerator::calculateYawPitch(
    const Eigen::Vector3d& armor_position,
    const Eigen::Vector3d& muzzle_position,
    double& yaw,
    double& pitch) const {

    Eigen::Vector3d relative_pos = armor_position - muzzle_position;

    yaw = std::atan2(relative_pos.y(), relative_pos.x());
    pitch = std::atan2(relative_pos.z(), relative_pos.head<2>().norm());
}

} // namespace rm_gimbal
