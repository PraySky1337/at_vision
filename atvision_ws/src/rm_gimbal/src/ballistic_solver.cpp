// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_gimbal/ballistic_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <stdexcept>
// ros2
#include <rclcpp/rclcpp.hpp>
// project
#include "rm_utils/math/utils.hpp"

namespace rm_gimbal {

BallisticSolver::BallisticSolver(const SolverParams& params, AtomicSolverParams& atomic_params)
    : atomic_params_(atomic_params)
    , shooting_range_w_(params.shooting_range_w)
    , shooting_range_h_(params.shooting_range_h)
    , transfer_thresh_(params.transfer_thresh) {

    // Create trajectory compensator
    trajectory_compensator_ = fyt::CompensatorFactory::createCompensator(params.compensator_type);
    if (!trajectory_compensator_) {
        throw std::runtime_error("invalid trajectory compensator type: " + params.compensator_type);
    }
    trajectory_compensator_->iteration_times = params.trajectory.iteration_times;
    trajectory_compensator_->velocity        = params.trajectory.bullet_speed;
    trajectory_compensator_->gravity         = params.trajectory.gravity;
    trajectory_compensator_->resistance      = params.trajectory.resistance;
}

void BallisticSolver::transitionTo(State new_state) {
    if (state != new_state) {
        overflow_count_ = 0;
        state           = new_state;
    }
}

rm_interfaces::msg::GimbalCmd BallisticSolver::solve(
    const rm_interfaces::msg::Target& target, const rclcpp::Time& current_time,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
    has_prediction_ = false;

    // Read atomic params (lock-free, no ROS parameter calls)
    const double max_tracking_v_yaw  = atomic_params_.max_tracking_v_yaw.load(std::memory_order_relaxed);
    const double prediction_delay    = atomic_params_.prediction_delay.load(std::memory_order_relaxed);

    // Get current roll, yaw and pitch of gimbal
    try {
        auto gimbal_tf =
            tf2_buffer_->lookupTransform(target.header.frame_id, "muzzle_link", tf2::TimePointZero);
        auto msg_q       = gimbal_tf.transform.rotation;
        auto translation = gimbal_tf.transform.translation;

        tf2::Quaternion tf_q;
        tf2::fromMsg(msg_q, tf_q);
        tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
        xyza_[0] = translation.x;
        xyza_[1] = translation.y;
        xyza_[2] = translation.z;
        xyza_[3] = 0.;

        rpy_[1] = -rpy_[1];
    } catch (tf2::TransformException& ex) {
        throw ex;
    }

    // Calculate target center position
    double target_z = target.armors_num == 3 ? (target.position.z + target.z1 + target.z2) / 3.0
                                             : (target.position.z + target.z1) / 2;
    Eigen::Vector3d target_position(target.position.x, target.position.y, target_z);
    double target_yaw = target.yaw;

    // Step 1: Initial flying time estimate based on target center
    double flying_time = trajectory_compensator_->getFlyingTime(target_position);
    rclcpp::Time target_stamp(target.header.stamp, current_time.get_clock_type());
    double dt = (current_time - target_stamp).seconds() + flying_time + prediction_delay;

    // Step 2: Predict target state
    target_position.x() += dt * target.velocity.x;
    target_position.y() += dt * target.velocity.y;
    target_position.z() += dt * target.velocity.z;
    target_yaw += dt * target.v_yaw;

    // Step 3: Get armor positions
    std::vector<Eigen::Vector4d> armor_pose;
    if (target.armors_num == 4) {
        armor_pose = util::get_robo_armor_poses(
            target_position, target_yaw, target.radius0, target.radius1, target.z1, target.z2,
            target.armors_num);
    } else if (target.armors_num == 3) {
        armor_pose = util::get_outpost_armor_poses(
            target_position, target_yaw, target.radius0, target.position.z, target.z1, target.z2);
    }

    // Step 4: Select best armor
    int idx = selectBestArmor(armor_pose, target_position, target_yaw, target.v_yaw, target.armors_num);

    // Step 5: Get chosen armor position relative to muzzle
    auto chosen_armor_pose_world      = armor_pose.at(idx);
    Eigen::Vector4d chosen_armor_pose = chosen_armor_pose_world - xyza_;
    Eigen::Vector3d armor_position{
        chosen_armor_pose.x(), chosen_armor_pose.y(), chosen_armor_pose.z()};
    Eigen::Vector3d predicted_position(
        chosen_armor_pose_world.x(), chosen_armor_pose_world.y(), chosen_armor_pose_world.z());
    double distance = chosen_armor_pose.norm();

    // Step 6: Refine flying time based on actual armor position (fixes systematic error)
    double refined_flying_time = trajectory_compensator_->getFlyingTime(armor_position);
    if (std::abs(refined_flying_time - flying_time) > 0.001) {
        // Re-predict with refined time
        double dt_diff = refined_flying_time - flying_time;
        target_position.x() += dt_diff * target.velocity.x;
        target_position.y() += dt_diff * target.velocity.y;
        target_position.z() += dt_diff * target.velocity.z;
        target_yaw += dt_diff * target.v_yaw;

        // Recalculate armor position
        if (target.armors_num == 4) {
            armor_pose = util::get_robo_armor_poses(
                target_position, target_yaw, target.radius0, target.radius1, target.z1, target.z2,
                target.armors_num);
        } else if (target.armors_num == 3) {
            armor_pose = util::get_outpost_armor_poses(
                target_position, target_yaw, target.radius0, target.position.z, target.z1, target.z2);
        }
        chosen_armor_pose_world = armor_pose.at(idx);
        chosen_armor_pose       = chosen_armor_pose_world - xyza_;
        armor_position          = Eigen::Vector3d(
            chosen_armor_pose.x(), chosen_armor_pose.y(), chosen_armor_pose.z());
        predicted_position = Eigen::Vector3d(
            chosen_armor_pose_world.x(), chosen_armor_pose_world.y(), chosen_armor_pose_world.z());
        distance = armor_position.norm();
    }

    // Step 7: Calculate yaw and pitch with ballistic compensation
    double yaw, pitch;
    calcYawAndPitch(armor_position, yaw, pitch);

    // Initialize gimbal_cmd
    rm_interfaces::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header      = target.header;
    gimbal_cmd.distance    = distance;
    gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);

    // State machine for tracking mode
    switch (state) {
    case TRACKING_ARMOR: {
        if (std::abs(target.v_yaw) > max_tracking_v_yaw) {
            overflow_count_++;
        } else {
            overflow_count_ = 0;
        }

        if (overflow_count_ > transfer_thresh_) {
            transitionTo(TRACKING_CENTER);
        }
        break;
    }
    case TRACKING_CENTER: {
        if (std::abs(target.v_yaw) < max_tracking_v_yaw) {
            overflow_count_++;
        } else {
            overflow_count_ = 0;
        }

        if (overflow_count_ > transfer_thresh_) {
            transitionTo(TRACKING_ARMOR);
        }
        gimbal_cmd.fire_advice = true;
        calcYawAndPitch(target_position, yaw, pitch);
        predicted_position = target_position;
        break;
    }
    }

    double cmd_pitch = pitch;
    double cmd_yaw   = angles::normalize_angle(yaw);

    gimbal_cmd.yaw        = cmd_yaw * 180 / M_PI;
    gimbal_cmd.pitch      = cmd_pitch * 180 / M_PI;
    gimbal_cmd.yaw_diff   = (cmd_yaw - rpy_[2]) * 180 / M_PI;
    gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;
    predicted_position_   = predicted_position;
    has_prediction_       = true;
    return gimbal_cmd;
}

bool BallisticSolver::isOnTarget(
    const double cur_yaw, const double cur_pitch, const double target_yaw,
    const double target_pitch, const double distance) const noexcept {
    // Judge whether to shoot
    double shooting_range_yaw   = std::abs(atan2(shooting_range_w_ / 2, distance));
    double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
    // Limit the shooting area to 1 degree to avoid not shooting when distance is too large
    shooting_range_yaw   = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
    shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
    if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw
        && std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
        return true;
    }

    return false;
}

int BallisticSolver::selectBestArmor(
    [[maybe_unused]] const std::vector<Eigen::Vector4d>& armor_positions,
    const Eigen::Vector3d& target_center,
    const double target_yaw, const double target_v_yaw, const size_t armors_num) const noexcept {

    const double side_angle          = atomic_params_.side_angle.load(std::memory_order_relaxed);
    const double min_switching_v_yaw = atomic_params_.min_switching_v_yaw.load(std::memory_order_relaxed);

    // Angle between the car's center and the X-axis
    double alpha = std::atan2(target_center.y(), target_center.x());
    // Angle between the front of observed armor and the X-axis
    double beta = target_yaw;

    // clang-format off
    Eigen::Matrix2d R_odom2center;
    Eigen::Matrix2d R_odom2armor;
    R_odom2center << std::cos(alpha), std::sin(alpha),
                  -std::sin(alpha), std::cos(alpha);
    R_odom2armor << std::cos(beta), std::sin(beta),
                 -std::sin(beta), std::cos(beta);
    // clang-format on
    Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

    // Use full angle to avoid aliasing when |alpha - beta| > 90 deg (outpost has 3 armors)
    double decision_angle = std::atan2(R_center2armor(1, 0), R_center2armor(0, 0));

    // Angle thresh of the armor jump
    double theta = (target_v_yaw > 0 ? side_angle : -side_angle) / 180.0 * M_PI;

    // Avoid the frequent switch between two armor
    if (std::abs(target_v_yaw) < min_switching_v_yaw) {
        theta = 0;
    }

    const double step = 2 * M_PI / static_cast<double>(armors_num);
    double temp_angle = decision_angle + step / 2 - theta;
    temp_angle        = std::fmod(temp_angle + 2 * M_PI, 2 * M_PI); // wrap to [0, 2π)
    int selected_id   = static_cast<int>(temp_angle / step);
    return selected_id;
}

void BallisticSolver::calcYawAndPitch(const Eigen::Vector3d& p, double& yaw, double& pitch) const noexcept {
    // Calculate yaw and pitch
    yaw   = atan2(p.y(), p.x());
    pitch = atan2(p.z(), p.head(2).norm());

    Eigen::Vector3d position = {p.x(), p.y(), p.z()};

    if (double temp_pitch = pitch; trajectory_compensator_->compensate(position, temp_pitch)) {
        pitch = temp_pitch;
    }
}

std::vector<std::pair<double, double>> BallisticSolver::getTrajectory() const noexcept {
    auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
    // Rotate
    for (auto& p : trajectory) {
        double x = p.first;
        double y = p.second;
        p.first  = x * cos(rpy_[1]) + y * sin(rpy_[1]);
        p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
    }
    return trajectory;
}

std::optional<Eigen::Vector3d> BallisticSolver::getPredictedPosition() const noexcept {
    if (!has_prediction_) {
        return std::nullopt;
    }
    return predicted_position_;
}

} // namespace rm_gimbal
