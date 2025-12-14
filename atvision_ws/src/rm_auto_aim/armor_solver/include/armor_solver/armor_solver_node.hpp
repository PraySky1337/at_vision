#pragma once

// ros2
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// std
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <vector>
// project
#include "armor_solver.hpp"
#include "tracker.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/measurement.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/logger/log.hpp"


namespace rm_auto_aim {
using tf2_filter = tf2_ros::MessageFilter<rm_interfaces::msg::Armors>;
class ArmorSolverNode : public rclcpp::Node {
public:
    explicit ArmorSolverNode(const rclcpp::NodeOptions& options);

private:
    void armorsCallback(rm_interfaces::msg::Armors::SharedPtr armors_ptr);

    void initMarkers() noexcept;

    Tracker::Params declareTrackerParameters();
    rcl_interfaces::msg::SetParametersResult
        onSetParameters(const std::vector<rclcpp::Parameter>& parameters);
    bool applyTrackerParamUpdate(
        const rclcpp::Parameter& param, Tracker::Params& params);

    void publishMarkers(
        const rm_interfaces::msg::Target& target_msg,
        const rm_interfaces::msg::GimbalCmd& gimbal_cmd) noexcept;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool debug_mode_;

    Tracker tracker_;
    Tracker::Params tracker_params_;
    // The time when the last message was received
    rclcpp::Time last_time_; // 保存这一刻的时间戳，阻止时间倒流
    // Always use system time regardless of /clock or use_sim_time
    rclcpp::Clock::SharedPtr system_clock_;
    double dt_;

    // Armor tracker
    double lost_time_thres_;

    // Armor Solver
    std::unique_ptr<Solver> solver_;
    std::mutex target_mutex_;
    std::mutex tracker_mutex_;

    // Subscriber with tf2 message_filter
    std::string target_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<rm_interfaces::msg::Armors> armors_sub_;
    rm_interfaces::msg::Target armor_target_;
    std::shared_ptr<tf2_filter> tf2_filter_;

    // Measurement publisher
    rclcpp::Publisher<rm_interfaces::msg::Measurement>::SharedPtr measure_pub_;

    // Publisher
    rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    void timerCallback();
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr tracker_param_cb_handle_;

    struct TrackerSnapshot {
        rm_interfaces::msg::Target target;
        bool valid{false};
    };

    // Visualization marker publisher
    visualization_msgs::msg::Marker position_marker_;
    visualization_msgs::msg::Marker linear_v_marker_;
    visualization_msgs::msg::Marker angular_v_marker_;
    visualization_msgs::msg::Marker trajectory_marker_;
    visualization_msgs::msg::Marker armors_marker_;
    visualization_msgs::msg::Marker selection_marker_;
    visualization_msgs::msg::Marker predicted_marker_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    TrackerSnapshot tracker_snapshot_;
};

} // namespace fyt::auto_aim
