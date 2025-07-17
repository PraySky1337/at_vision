#pragma once
#include "usb.hpp"
#include "usb/packet.hpp"

#include <auto_aim_interfaces/msg/detail/control_cmd__struct.hpp>
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "auto_aim_interfaces/msg/control_cmd.hpp"
#include "util/atmath.hpp"

#include <cstdint>

namespace usb_driver {
struct UsbDriver : public rclcpp::Node {
    enum Color { RED, BLUE, UNKNOWN };
    UsbDriver(
        const rclcpp::NodeOptions& options)
        : rclcpp::Node("usb_driver", options)
        , device_(parser_)
        , tf_broadcaster_(*this)
        , aiming_color_(UNKNOWN) {
        tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_    = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        detector_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
        reset_tracker_srv = create_client<std_srvs::srv::Trigger>("reset_tracker");
        this->init_parser();
        if (device_.open(0x0483)) {
            ATLOG_INFO("usb driver already");
        }
        control_cmd_sub_ = create_subscription<auto_aim_interfaces::msg::ControlCmd>(
            "trajectory/control_command", rclcpp::SensorDataQoS(),
            std::bind(&UsbDriver::control_cmd_callback, this, std::placeholders::_1));
        thread_ = std::thread([this] {
            running_ = true;
            while (running_) {
                device_.handle_events();
            }
        });
    }

    ~UsbDriver() {
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
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
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now() - rclcpp::Duration::from_seconds(0.01);
        ;
        t.header.frame_id    = "odom";
        t.child_frame_id     = "gimbal_link";
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_.sendTransform(t);
        if (d.self_color != aiming_color_) {}
    }

    void control_cmd_callback(
        const auto_aim_interfaces::msg::ControlCmd::SharedPtr control_cmd_msg) {
        SendVisionData vision_data;
        vision_data.header.id           = 0x02;
        vision_data.header.len          = sizeof(decltype(vision_data.data));
        vision_data.header.sof          = HeaderFrame::SoF();
        vision_data.eof                 = HeaderFrame::EoF();
        vision_data.data.tracking       = control_cmd_msg->tracking;
        vision_data.data.target_pitch   = control_cmd_msg->target_pitch;
        vision_data.data.target_yaw     = control_cmd_msg->target_yaw;
        vision_data.data.is_large_armor = control_cmd_msg->is_large_armor;
        std::memcpy(buffer_, &vision_data, sizeof(SendVisionData));
        if (!device_.send_data(buffer_, sizeof(SendVisionData))) {
            ATLOG_WARN("Failed to send data");
        }
        // 打印 buffer 内容（16进制）
        // std::ostringstream oss;
        // oss << "Send buffer: ";
        // for (size_t i = 0; i < sizeof(SendVisionData); ++i) {
        //     oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
        //         << (static_cast<unsigned>(buffer_[i]) & 0xFF) << " ";
        // }
        // ATLOG_INFO("{}", oss.str());
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

    rclcpp::Subscription<auto_aim_interfaces::msg::ControlCmd>::SharedPtr control_cmd_sub_;

    std::atomic_bool running_;
    std::thread thread_;
};
} // namespace usb_driver
