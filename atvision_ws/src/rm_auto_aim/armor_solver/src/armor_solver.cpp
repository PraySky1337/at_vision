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

#include "armor_solver/armor_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <stdexcept>
// project
#include "armor_solver/armor_solver_node.hpp"
#include "armor_solver/rk4_compensator.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace rm_auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n)
    : node_(n) {
    auto node = node_.lock();

    shooting_range_w_    = node->declare_parameter("solver.shooting_range_width", 0.135);
    shooting_range_h_    = node->declare_parameter("solver.shooting_range_height", 0.135);
    max_tracking_v_yaw_  = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
    prediction_delay_    = node->declare_parameter("solver.prediction_delay", 0.0);
    controller_delay_    = node->declare_parameter("solver.controller_delay", 0.0);
    side_angle_          = node->declare_parameter("solver.side_angle", 15.0);
    min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);

    std::string compenstator_type = node->declare_parameter("solver.compensator_type", "rk4");
    if (compenstator_type == "rk4") {
        trajectory_compensator_ = std::make_unique<Rk4Compensator>();
    } else {
        trajectory_compensator_ = fyt::CompensatorFactory::createCompensator(compenstator_type);
    }
    if (!trajectory_compensator_) {
        throw std::runtime_error("invalid trajectory compensator type: " + compenstator_type);
    }
    trajectory_compensator_->iteration_times =
        node->declare_parameter("solver.iteration_times", 20);
    trajectory_compensator_->velocity   = node->declare_parameter("solver.bullet_speed", 20.0);
    trajectory_compensator_->gravity    = node->declare_parameter("solver.gravity", 9.8);
    trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);

    auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string>{});
    overflow_count_   = 0;
    transfer_thresh_  = 5;

    node.reset();
}

rm_interfaces::msg::GimbalCmd Solver::solve(
    const rm_interfaces::msg::Target& target, const rclcpp::Time& current_time,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
    has_prediction_ = false;
    // Get newest parameters
    try {
        auto node            = node_.lock();
        max_tracking_v_yaw_  = node->get_parameter("solver.max_tracking_v_yaw").as_double();
        prediction_delay_    = node->get_parameter("solver.prediction_delay").as_double();
        controller_delay_    = node->get_parameter("solver.controller_delay").as_double();
        side_angle_          = node->get_parameter("solver.side_angle").as_double();
        min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
        node.reset();
    } catch (const std::runtime_error& e) {
        // nothing
    }

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

    // Use flying time to approximately predict the position of target
    double target_z = target.armors_num == 3 ? (target.position.z + target.z1 + target.z2) / 3.0
                                             : (target.position.z + target.z1) / 2;
    Eigen::Vector3d target_position(target.position.x, target.position.y, target_z);
    double target_yaw  = target.yaw;
    double flying_time = trajectory_compensator_->getFlyingTime(target_position);
    rclcpp::Time target_stamp(target.header.stamp, current_time.get_clock_type());
    double dt = (current_time - target_stamp).seconds() + flying_time + prediction_delay_;
    target_position.x() += dt * target.velocity.x;
    target_position.y() += dt * target.velocity.y;
    target_position.z() += dt * target.velocity.z;
    target_yaw += dt * target.v_yaw;

    // Choose the best armor to shoot
    std::vector<Eigen::Vector4d> armor_pose(4);
    if (target.armors_num == 4) {
        armor_pose = util::get_robo_armor_poses(
            target_position, target_yaw, target.radius0, target.radius1, target.z1, target.z2,
            target.armors_num);
    } else if (target.armors_num == 3) {
        armor_pose = util::get_outpost_armor_poses(
            target_position, target_yaw, target.radius0, target.position.z, target.z1, target.z2);
    }

    int idx =
        selectBestArmor(armor_pose, target_position, target_yaw, target.v_yaw, target.armors_num);
    auto chosen_armor_pose_world      = armor_pose.at(idx);
    Eigen::Vector4d chosen_armor_pose = chosen_armor_pose_world - xyza_;
    Eigen::Vector3d armor_position{
        chosen_armor_pose.x(), chosen_armor_pose.y(), chosen_armor_pose.z()};
    Eigen::Vector3d predicted_position(
        chosen_armor_pose_world.x(), chosen_armor_pose_world.y(), chosen_armor_pose_world.z());
    double distance = chosen_armor_pose.norm();

    // Calculate yaw, pitch, distance
    double yaw, pitch;
    calcYawAndPitch(armor_position, yaw, pitch);

    // Initialize gimbal_cmd
    rm_interfaces::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header      = target.header;
    gimbal_cmd.distance    = distance;
    gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);

    switch (state) {
    case TRACKING_ARMOR: {
        if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
            overflow_count_++;
        } else {
            overflow_count_ = 0;
        }

        if (overflow_count_ > transfer_thresh_) {
            state = TRACKING_CENTER;
        }

        // If isOnTarget() never returns true, adjust controller_delay to force the gimbal to move
        if (controller_delay_ != 0) {
            target_position.x() += controller_delay_ * target.velocity.x;
            target_position.y() += controller_delay_ * target.velocity.y;
            target_position.z() += controller_delay_ * target.velocity.z;
            target_yaw += controller_delay_ * target.v_yaw;
            armor_pose              = util::get_robo_armor_poses(target);
            chosen_armor_pose_world = armor_pose.at(idx);
            chosen_armor_pose       = chosen_armor_pose_world - xyza_;
            armor_position          = Eigen::Vector3d(
                chosen_armor_pose.x(), chosen_armor_pose.y(), chosen_armor_pose.z());
            predicted_position = Eigen::Vector3d(
                chosen_armor_pose_world.x(), chosen_armor_pose_world.y(),
                chosen_armor_pose_world.z());
            gimbal_cmd.distance = armor_position.norm();
            calcYawAndPitch(armor_position, yaw, pitch);
        }
        break;
    }
    case TRACKING_CENTER: {
        if (std::abs(target.v_yaw) < max_tracking_v_yaw_) {
            overflow_count_++;
        } else {
            overflow_count_ = 0;
        }

        if (overflow_count_ > transfer_thresh_) {
            state           = TRACKING_ARMOR;
            overflow_count_ = 0;
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

bool Solver::isOnTarget(
    const double cur_yaw, const double cur_pitch, const double target_yaw,
    const double target_pitch, const double distance) const noexcept {
    // Judge whether to shoot
    double shooting_range_yaw   = std::abs(atan2(shooting_range_w_ / 2, distance));
    double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
    // Limit the shooting area to 1 degree to avoid not shooting when distance is
    // too large
    shooting_range_yaw   = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
    shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
    if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw
        && std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
        return true;
    }

    return false;
}

int Solver::selectBestArmor(
    const std::vector<Eigen::Vector4d>& armor_positions, const Eigen::Vector3d& target_center,
    const double target_yaw, const double target_v_yaw, const size_t armors_num) const noexcept {
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
    double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

    // Avoid the frequent switch between two armor
    if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
        theta = 0;
    }

    const double step = 2 * M_PI / static_cast<double>(armors_num);
    double temp_angle = decision_angle + step / 2 - theta;
    temp_angle        = std::fmod(temp_angle + 2 * M_PI, 2 * M_PI); // wrap to [0, 2π)
    int selected_id   = static_cast<int>(temp_angle / step);
    return selected_id;
}

void Solver::calcYawAndPitch(const Eigen::Vector3d& p, double& yaw, double& pitch) const noexcept {
    // Calculate yaw and pitch
    yaw   = atan2(p.y(), p.x());
    pitch = atan2(p.z(), p.head(2).norm());

    Eigen::Vector3d position = {p.x(), p.y(), p.z()};

    if (double temp_pitch = pitch; trajectory_compensator_->compensate(position, temp_pitch)) {
        pitch = temp_pitch;
    }
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
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

std::optional<Eigen::Vector3d> Solver::getPredictedPosition() const noexcept {
    if (!has_prediction_) {
        return std::nullopt;
    }
    return predicted_position_;
}

} // namespace rm_auto_aim
