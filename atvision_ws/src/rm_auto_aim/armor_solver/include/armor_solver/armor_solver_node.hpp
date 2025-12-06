#pragma once

// ros2
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// std
#include <memory>
#include <string>
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


namespace fyt::auto_aim {
using tf2_filter = tf2_ros::MessageFilter<rm_interfaces::msg::Armors>;
class ArmorSolverNode : public rclcpp::Node {
public:
    explicit ArmorSolverNode(const rclcpp::NodeOptions& options);

private:
    void armorsCallback(rm_interfaces::msg::Armors::SharedPtr armors_ptr);

    void initMarkers() noexcept;

    void publishMarkers(
        const rm_interfaces::msg::Target& target_msg,
        const rm_interfaces::msg::GimbalCmd& gimbal_cmd) noexcept;
    // demo
    void publish_cv_3d_markers(const rm_interfaces::msg::Target& target) noexcept;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool debug_mode_;

    armor_tracker::Tracker tracker_;
    armor_tracker::Tracker::Params tracker_params_;
    // The time when the last message was received
    rclcpp::Time last_time_;
    double dt_;

    // Armor tracker
    double s2qxyz_, s2qyaw_, s2qr_, s2qd_zc_;
    double r_x_, r_y_, r_z_, r_yaw_;
    double lost_time_thres_;

    // Armor Solver
    std::unique_ptr<Solver> solver_;
    std::mutex target_mutex_;

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

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    TrackerSnapshot tracker_snapshot_;
};

} // namespace fyt::auto_aim
