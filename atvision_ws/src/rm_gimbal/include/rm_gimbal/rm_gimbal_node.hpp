#pragma once

#include "packet.hpp"
#include "usb.hpp"

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

#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "rm_interfaces/msg/gimbal_cmd.hpp"

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

    void control_cmd_callback(rm_interfaces::msg::GimbalCmd::ConstSharedPtr plan_control_cmd_msg);

private:
    DeviceParser parser_;
    Device device_;
    uint8_t buffer_[64];

    tf2_ros::Buffer::SharedPtr tf_buffer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::CallbackGroup::SharedPtr call_back_group;

    std::atomic<uint8_t> aiming_color_;

    rclcpp::AsyncParametersClient::SharedPtr detector_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv;

    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr control_cmd_sub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_cb_;

    std::atomic<bool> running_;
    std::thread thread_;

    std::atomic<double> timestamp_offset_ms_;
    bool debug_;
    bool use_roll_;
    bool use_planner_;
};

} // namespace rm_gimbal
