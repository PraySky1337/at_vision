#pragma once

#include "packet.hpp"
#include "usb.hpp"
#include "rm_gimbal/ballistic_solver.hpp"
#include "rm_gimbal/solver_params.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/plan_gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/energy_meter_state.hpp"

namespace rm_gimbal {

struct GimbalNode : public rclcpp::Node {
    enum class EnemyColor { RED = 0, BLUE = 1, UNKNOWN = 2};

    explicit GimbalNode(const rclcpp::NodeOptions& options);
    ~GimbalNode();

private:
    rcl_interfaces::msg::SetParametersResult
    on_params(const std::vector<rclcpp::Parameter>& params);

    void init_parser();

    void handle_imu_packet(const std::byte* data, size_t size);

    // Target callback - receives tracking target from armor_solver
    void target_callback(rm_interfaces::msg::Target::ConstSharedPtr target_msg);

    // Solve timer callback - 250Hz ballistic solving
    void solve_timer_callback();

    // Send gimbal command directly to device
    void send_to_device(const rm_interfaces::msg::GimbalCmd& cmd);

    // Send empty command when not tracking
    void send_empty_command();

    void set_params(const std::string& color);

    // Declare solver parameters
    SolverParams declare_solver_parameters();

    // Apply solver parameter updates (atomic, for hot-reload)
    bool apply_solver_param_update(const rclcpp::Parameter& param);

    // Initialize visualization markers
    void init_markers() noexcept;

    // Publish visualization markers
    void publish_markers(
        const rm_interfaces::msg::Target& target_msg,
        const rm_interfaces::msg::GimbalCmd& gimbal_cmd) noexcept;

private:
    DeviceParser parser_;
    Device device_;
    uint8_t buffer_[64];

    tf2_ros::Buffer::SharedPtr tf_buffer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::atomic<uint8_t> aiming_color_;

    rclcpp::AsyncParametersClient::SharedPtr detector_client_;

    // Target subscription (replaces GimbalCmd subscription)
    rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr target_sub_;
    rm_interfaces::msg::Target armor_target_;
    std::mutex target_mutex_;

    // Rune marker subscription (energy mechanism predicted position)
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr rune_marker_sub_;
    void rune_marker_callback(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg);

    // Energy meter state subscription (for MPC)
    rclcpp::Subscription<rm_interfaces::msg::EnergyMeterState>::SharedPtr energy_state_sub_;
    void energy_state_callback(rm_interfaces::msg::EnergyMeterState::ConstSharedPtr msg);
    rm_interfaces::msg::EnergyMeterState cached_energy_state_;
    std::mutex energy_state_mutex_;
    bool energy_state_valid_{false};

    // Ballistic solver
    std::unique_ptr<BallisticSolver> solver_;
    SolverParams solver_params_;
    AtomicSolverParams atomic_solver_params_;

    // Solve timer (250Hz)
    rclcpp::TimerBase::SharedPtr solve_timer_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    // Visualization
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    visualization_msgs::msg::Marker trajectory_marker_;
    visualization_msgs::msg::Marker selection_marker_;
    visualization_msgs::msg::Marker predicted_marker_;

    // Debug publisher for GimbalCmd
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;

    // Debug publisher for PlanGimbalCmd (MPC planning data)
    rclcpp::Publisher<rm_interfaces::msg::PlanGimbalCmd>::SharedPtr plan_cmd_pub_;

    // Publisher for ballistic params (flying_time, prediction_delay) to energy_meter_solver
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ballistic_params_pub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_cb_;

    // usb thread
    std::atomic<bool> running_;
    std::thread thread_;

    // flag
    std::atomic<double> timestamp_offset_ms_;
    bool debug_;
    bool use_roll_;
    bool use_simulator_;  // true: 模拟器模式，不连接下位机
};

} // namespace rm_gimbal
