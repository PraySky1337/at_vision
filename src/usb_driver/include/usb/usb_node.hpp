#pragma once
#include "usb.hpp"
#include "usb/packet.hpp"

#include <memory>
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
#include "rm_interfaces/msg/plan_control_cmd.hpp"

#include <cstdint>

namespace usb_driver {
struct UsbDriver : public rclcpp::Node {
    enum Color { RED, BLUE, UNKNOWN };
    UsbDriver(const rclcpp::NodeOptions& options)
        : rclcpp::Node("usb_driver", options)
        , device_(parser_)
        , tf_broadcaster_(*this)
        , aiming_color_(UNKNOWN) {
        tf_buffer_        = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_      = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        detector_client   = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
        reset_tracker_srv = create_client<std_srvs::srv::Trigger>("reset_tracker");
        this->init_parser();
        if (device_.open(0x0483)) {
            ATLOG_INFO("usb driver already");
        }
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.description = "unit: ms";
        timestamp_offset_ms_   = this->declare_parameter("timestamp_offset", 0.1, param_desc); // s
        control_cmd_sub_       = create_subscription<rm_interfaces::msg::GimbalCmd>(
            "armor_solver/cmd_gimbal", rclcpp::SensorDataQoS(),
            std::bind(&UsbDriver::control_cmd_callback, this, std::placeholders::_1));
        on_set_params_cb_ = add_on_set_parameters_callback(
            std::bind(&UsbDriver::on_params, this, std::placeholders::_1));
        thread_ = std::thread([this] {
            running_ = true;
            while (running_) {
                device_.handle_events();
            }
        });
    }

    ~UsbDriver() {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    rcl_interfaces::msg::SetParametersResult
        on_params(const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        res.reason     = "";
        for (const auto& p : params)
            try {
                auto& name = p.get_name();
                if (name == "timestamp_offset") {
                    timestamp_offset_ms_.store(p.as_double());
                } else {
                }
            } catch (const rclcpp::ParameterTypeException& e) {
                RCLCPP_ERROR(get_logger(), "Parameter type error: %s", e.what());
            } catch (...) {
                RCLCPP_FATAL(get_logger(), "参数热更新回调时发生未知错误");
            }
        return res;
    }

    void init_parser() {
        parser_.register_parser(
            0x01,
            std::bind(
                &UsbDriver::handle_imu_packet, this, std::placeholders::_1, std::placeholders::_2));
    }

    void handle_imu_packet(const std::byte* data, size_t size) {
        if (size < sizeof(ReceiveImuData)) [[unlikely]] {
            ATLOG_WARN(
                "IMU packet too small, expected: {:X}, got {:X}", sizeof(ReceiveImuData), size);
            return;
        }

        ReceiveImuData imu_pkt;
        std::memcpy(&imu_pkt, data, sizeof(ReceiveImuData));
        tf2::Quaternion q;
        const auto& d = imu_pkt.data;
        q.setRPY(d.roll, d.pitch, d.yaw);
        q.normalize();
        if (std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z())) [[unlikely]] {
            ATLOG_WARN("roll, pitch or yaw is invalid nan");
        }
        double offset_ms = timestamp_offset_ms_.load();
        auto duration    = rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double, std::milli>(offset_ms)));
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp       = now() - duration;
        t.header.frame_id    = "odom";
        t.child_frame_id     = "gimbal_link";
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_.sendTransform(t);
        if (d.self_color != aiming_color_) {}
    }

    void control_cmd_callback(
        const rm_interfaces::msg::GimbalCmd::ConstSharedPtr plan_control_cmd_msg) {
        SendVisionData vision_data;
        vision_data.header.id         = 0x02;
        vision_data.header.len        = sizeof(decltype(vision_data.data));
        vision_data.header.sof        = HeaderFrame::SoF();
        vision_data.eof               = HeaderFrame::EoF();
        vision_data.data.fire_advice  = plan_control_cmd_msg->fire_advice;
        vision_data.data.target_pitch = -plan_control_cmd_msg->pitch;
        vision_data.data.target_yaw   = plan_control_cmd_msg->yaw;
        vision_data.data.distance     = plan_control_cmd_msg->distance;
        std::memcpy(buffer_, &vision_data, sizeof(SendVisionData));
        if (!device_.send_data(buffer_, sizeof(SendVisionData))) {
            ATLOG_WARN("Failed to send data");
        }
    }
    static constexpr const int DEV_VID = 0x0483;
    DeviceParser parser_;
    Device device_;
    uint8_t buffer_[64];

    tf2_ros::Buffer::SharedPtr tf_buffer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::CallbackGroup::SharedPtr call_back_group;

    std::atomic<uint8_t> aiming_color_;

    std::mutex param_mutex_;
    rclcpp::AsyncParametersClient::SharedPtr detector_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv;

    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr control_cmd_sub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_cb_;

    std::atomic<bool> running_;
    std::thread thread_;

    std::atomic<double> timestamp_offset_ms_;
};
} // namespace usb_driver
