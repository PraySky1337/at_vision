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
#include "rm_gimbal/solver_params.hpp"
#include "rm_gimbal/trajectory_generator.hpp"
#include "rm_gimbal/trajectory_planner.hpp"
#include "rm_gimbal/util.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"

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

    // Solve the gimbal command for rune target (energy mechanism) with MPC support
    rm_interfaces::msg::GimbalCmd solveRune(
        const Eigen::Vector3d& target_position, const Eigen::Vector3d& r_center, double radius,
        const Eigen::Vector3d& rotation_u, const Eigen::Vector3d& rotation_v,
        double angular_velocity, const std_msgs::msg::Header& header,
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);

    enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state{TRACKING_ARMOR};

    std::vector<std::pair<double, double>> getTrajectory() const noexcept;

    std::optional<Eigen::Vector3d> getPredictedPosition() const noexcept;

    // 获取弹道参数（用于传递给能量机关求解器）
    double getLastFlyingTime() const noexcept { return last_flying_time_; }
    double getLastPredictionDelay() const noexcept { return last_prediction_delay_; }

    // 获取MPC前馈量
    FeedforwardCache getFeedforward() const noexcept { return feedforward_cache_; }

    // 初始化MPC（需要在参数加载后调用）
    bool initializeMPC();

    // 更新MPC参数
    void updateMPCParams(const MPCParams& params);

    // 设置云台角速度（从IMU获取）
    void setGimbalAngularVelocity(double yaw_vel, double pitch_vel) noexcept;

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

    // MPC轨迹规划模块
    std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
    std::unique_ptr<TrajectoryPlanner> trajectory_planner_;
    FeedforwardCache feedforward_cache_;
    MPCParams mpc_params_;
    bool mpc_enabled_{false};

    // 云台角速度（从IMU获取）
    double gimbal_yaw_vel_{0.0};
    double gimbal_pitch_vel_{0.0};

    // 弹道参数缓存（用于传递给能量机关求解器）
    double last_flying_time_{0.0};
    double last_prediction_delay_{0.0};
};

} // namespace rm_gimbal

#endif // RM_GIMBAL_BALLISTIC_SOLVER_HPP_
