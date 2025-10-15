#include "rm_gimbal/rm_gimbal_node.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <utility>

#include <rclcpp/duration.hpp>

namespace rm_gimbal {

GimbalNode::GimbalNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("gimbal_node", options)
    , device_(parser_)
    , tf_broadcaster_(*this)
    , aiming_color_(static_cast<int>(EnemyColor::UNKNOWN)) {
    tf_buffer_        = std::make_shared<tf2_ros::Buffer>(get_clock());
    detector_client   = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
    reset_tracker_srv = create_client<std_srvs::srv::Trigger>("reset_tracker");
    this->init_parser();
    try {
        if (device_.open(0x0483)) {
            RCLCPP_INFO(get_logger(), "Gimbal Node already");
        } else {
            RCLCPP_FATAL(get_logger(), "Failed to open gimbal device");
        }
    } catch (...) {
        RCLCPP_FATAL(get_logger(), "Failed to open gimbal device");
        device_.open(0x0483); // 再尝试一次 如果未成功则再次触发异常
    }

    use_roll_    = declare_parameter("use_roll", false);
    use_planner_ = declare_parameter("use_planner", true);
    debug_       = declare_parameter("debug", true);
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.description = "unit: ms";
    timestamp_offset_ms_   = this->declare_parameter("timestamp_offset", 0.1, param_desc); // s

    control_cmd_sub_ = create_subscription<rm_interfaces::msg::GimbalCmd>(
        "armor_solver/cmd_gimbal", rclcpp::SensorDataQoS(),
        std::bind(&GimbalNode::control_cmd_callback, this, std::placeholders::_1));

    on_set_params_cb_ = add_on_set_parameters_callback(
        std::bind(&GimbalNode::on_params, this, std::placeholders::_1));

    thread_ = std::thread([this] {
        running_ = true;
        while (running_) {
            device_.handle_events();
        }
    });
}

GimbalNode::~GimbalNode() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

rcl_interfaces::msg::SetParametersResult
    GimbalNode::on_params(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    res.reason     = "";
    for (const auto& p : params) {
        try {
            const auto& name = p.get_name();
            if (name == "timestamp_offset") {
                timestamp_offset_ms_.store(p.as_double());
            } else {
                // 保持逻辑不变：其他参数忽略
            }
        } catch (const rclcpp::ParameterTypeException& e) {
            RCLCPP_ERROR(get_logger(), "Parameter type error: %s", e.what());
        } catch (...) {
            RCLCPP_FATAL(get_logger(), "参数热更新回调时发生未知错误");
        }
    }
    return res;
}

void GimbalNode::init_parser() {
    parser_.register_parser(
        0x01,
        std::bind(
            &GimbalNode::handle_imu_packet, this, std::placeholders::_1, std::placeholders::_2));
}

void GimbalNode::handle_imu_packet(const std::byte* data, size_t size) {
    if (size < sizeof(ReceiveImuData)) [[unlikely]] {
        RCLCPP_WARN_STREAM(
            this->get_logger(), "IMU packet too small, expected: 0x" << std::uppercase << std::hex
                                                                     << sizeof(ReceiveImuData)
                                                                     << ", got: 0x" << size);
        return;
    }

    ReceiveImuData imu_pkt;
    std::memcpy(&imu_pkt, data, sizeof(ReceiveImuData));

    tf2::Quaternion q;
    const auto& d = imu_pkt.data;
    float roll    = use_roll_ ? d.roll : 0.f;
    q.setRPY(roll, d.pitch, d.yaw);
    q.normalize();
    if (std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z())) [[unlikely]] {
        RCLCPP_WARN(get_logger(), "roll, pitch or yaw is invalid nan");
    }

    double offset_ms = timestamp_offset_ms_.load();
    auto duration    = rclcpp::Duration(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double, std::milli>(offset_ms)));

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp       = now() - duration;
    t.header.frame_id    = "odom";
    t.child_frame_id     = "gimbal_link";
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_.sendTransform(t);

    if (d.self_color != aiming_color_) {
        // 原逻辑：占位，不做任何事
    }
}

void GimbalNode::control_cmd_callback(
    rm_interfaces::msg::GimbalCmd::ConstSharedPtr control_cmd_msg) {
    SendVisionData vision_data;
    vision_data.header.id         = 0x02;
    vision_data.header.len        = sizeof(decltype(vision_data.data));
    vision_data.header.sof        = HeaderFrame::SoF();
    vision_data.eof               = HeaderFrame::EoF();
    vision_data.data.fire_advice  = control_cmd_msg->fire_advice;
    vision_data.data.distance     = static_cast<float>(control_cmd_msg->distance);
    vision_data.data.target_pitch = -static_cast<float>(control_cmd_msg->pitch);
    vision_data.data.target_yaw   = static_cast<float>(control_cmd_msg->yaw);
    vision_data.data.ref_yaw_v    = 0.;
    vision_data.data.ref_pitch_v  = 0.;
    vision_data.data.ref_yaw_a    = 0.;
    vision_data.data.ref_pitch_a  = 0.;

    std::memcpy(buffer_, &vision_data, sizeof(SendVisionData));
    if (!device_.send_data(buffer_, sizeof(SendVisionData))) {
        RCLCPP_WARN(get_logger(), "Failed to send data");
    }
}

} // namespace rm_gimbal

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_gimbal::GimbalNode)
