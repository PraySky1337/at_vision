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

#ifndef RM_GIMBAL_BALLISTIC_SOLVER_HPP_
#define RM_GIMBAL_BALLISTIC_SOLVER_HPP_

// std
#include <memory>
#include <optional>
// ros2
#include <angles/angles.h>
#include <tf2_ros/buffer.h>

#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// 3rd party
#include <Eigen/Dense>
// project
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"
#include "rm_gimbal/solver_params.hpp"
#include "rm_gimbal/util.hpp"

namespace rm_gimbal {

// Solver class used to solve the gimbal command from tracked target
class BallisticSolver {
public:
    // Construct with initial parameters and atomic params reference for hot-reload
    explicit BallisticSolver(const SolverParams& params, AtomicSolverParams& atomic_params);
    ~BallisticSolver() = default;

    // Solve the gimbal command from tracked target
    // Throw: tf2::TransformException if the transform from "odom" to "gimbal_link" is not available
    rm_interfaces::msg::GimbalCmd solve(
        const rm_interfaces::msg::Target& target_msg, const rclcpp::Time& current_time,
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);

    enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state{TRACKING_ARMOR};

    std::vector<std::pair<double, double>> getTrajectory() const noexcept;

    std::optional<Eigen::Vector3d> getPredictedPosition() const noexcept;

private:
    // State transition helper - resets overflow_count on any state change
    void transitionTo(State new_state);

    // Select the best armor to shoot
    // Return: selected idx in {0, 1, ..., armors_num - 1}
    int selectBestArmor(
        const std::vector<Eigen::Vector4d>& armor_positions, const Eigen::Vector3d& target_center,
        double target_yaw, double target_v_yaw, size_t armors_num) const noexcept;

    void calcYawAndPitch(const Eigen::Vector3d& p, double& yaw, double& pitch) const noexcept;

    bool isOnTarget(
        double cur_yaw, double cur_pitch, double target_yaw, double target_pitch,
        double distance) const noexcept;

    std::unique_ptr<fyt::TrajectoryCompensator> trajectory_compensator_;

    Eigen::Vector3d rpy_;
    Eigen::Vector4d xyza_;

    // Reference to atomic params for hot-reload (no get_parameter calls in solve())
    AtomicSolverParams& atomic_params_;

    // Immutable params (set at construction)
    double shooting_range_w_;
    double shooting_range_h_;
    int transfer_thresh_;

    // State machine
    int overflow_count_{0};

    Eigen::Vector3d predicted_position_{Eigen::Vector3d::Zero()};
    bool has_prediction_{false};
};

} // namespace rm_gimbal

#endif // RM_GIMBAL_BALLISTIC_SOLVER_HPP_
