// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
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

#include "armor_solver/armor_solver_node.hpp"

// std
#include <cmath>
#include <memory>
#include <optional>
#include <vector>
// project
#include "rm_utils/common.hpp"
#include "rm_utils/heartbeat.hpp"

namespace {
rcl_interfaces::msg::ParameterDescriptor
    makeIntDescriptor(const std::string& description, int from, int to, int step = 1) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = description;
    desc.integer_range.resize(1);
    desc.integer_range[0].from_value = from;
    desc.integer_range[0].to_value   = to;
    desc.integer_range[0].step       = step;
    return desc;
}

rcl_interfaces::msg::ParameterDescriptor makeDoubleDescriptor(
    const std::string& description, double from, double to, double step = 0.01) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = description;
    desc.floating_point_range.resize(1);
    desc.floating_point_range[0].from_value = from;
    desc.floating_point_range[0].to_value   = to;
    desc.floating_point_range[0].step       = step;
    return desc;
}
} // namespace

namespace rm_auto_aim {

ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions& options)
    : Node("armor_solver", options)
    , system_clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
    , dt_(0.004) {
    // Register logger
    RCLCPP_INFO(get_logger(), "Starting ArmorSolver node (tracker only, solver moved to rm_gimbal)");

    debug_mode_ = this->declare_parameter("debug", true);

    // Subscriber with tf2 message_filter
    tf2_buffer_          = std::make_shared<tf2_ros::Buffer>(system_clock_);
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    armors_sub_.subscribe(this, "armor_detector/armors", rmw_qos_profile_sensor_data);
    target_frame_ = this->declare_parameter("target_frame", "odom");
    tf2_filter_   = std::make_shared<tf2_filter>(
        armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
        this->get_node_clock_interface(), std::chrono::duration<int>(1));
    tf2_filter_->registerCallback(&ArmorSolverNode::armorsCallback, this);

    // Measurement publisher (for debug usage)
    measure_pub_ = this->create_publisher<rm_interfaces::msg::Measurement>(
        "armor_solver/measurement", rclcpp::SensorDataQoS());

    // Publisher
    target_pub_ = this->create_publisher<rm_interfaces::msg::Target>(
        "armor_solver/target", rclcpp::SensorDataQoS());
    marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_solver/marker", 10);

    tracker_params_ = declareTrackerParameters();
    tracker_.set_params(tracker_params_);

    tracker_param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ArmorSolverNode::onSetParameters, this, std::placeholders::_1));

    initMarkers();
}

void ArmorSolverNode::initMarkers() noexcept {
    // Visualization Marker Publisher
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    position_marker_.ns      = "position";
    position_marker_.type    = visualization_msgs::msg::Marker::SPHERE;
    position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;

    position_marker_.color.a  = 0.3;
    position_marker_.color.g  = 1.0;
    linear_v_marker_.type     = visualization_msgs::msg::Marker::ARROW;
    linear_v_marker_.ns       = "linear_v";
    linear_v_marker_.scale.x  = 0.03;
    linear_v_marker_.scale.y  = 0.05;
    linear_v_marker_.color.a  = 0.3;
    linear_v_marker_.color.r  = 1.0;
    linear_v_marker_.color.g  = 1.0;
    angular_v_marker_.type    = visualization_msgs::msg::Marker::ARROW;
    angular_v_marker_.ns      = "angular_v";
    angular_v_marker_.scale.x = 0.03;
    angular_v_marker_.scale.y = 0.05;
    angular_v_marker_.color.a = 0.3;
    angular_v_marker_.color.b = 1.0;
    angular_v_marker_.color.g = 1.0;
    armors_marker_.ns         = "filtered_armors";
    armors_marker_.type       = visualization_msgs::msg::Marker::CUBE;
    armors_marker_.scale.x    = 0.03;
    armors_marker_.scale.z    = 0.125;
    armors_marker_.color.a    = 0.3;
    armors_marker_.color.b    = 1.0;
}

Tracker::Params ArmorSolverNode::declareTrackerParameters() {
    Tracker::Params params{};
    auto& rbp = params.robot_params;
    auto& opp = params.outpost_params;

// 定义中文参数声明宏
#define DECLARE_INT_ZH(name, default_val, desc_zh, min, max) \
    this->declare_parameter(name, default_val, makeIntDescriptor(desc_zh, min, max))

#define DECLARE_DOUBLE_ZH(name, default_val, desc_zh, min, max, step) \
    this->declare_parameter(name, default_val, makeDoubleDescriptor(desc_zh, min, max, step))

    // clang-format off
    params.lost_thres = DECLARE_DOUBLE_ZH("tracker.lost_threshold", 1.0,
        "丢失阈值：TEMP_LOST持续时间超过此值（秒）则重置跟踪器", 0.0, 10.0, 0.001);
    params.tracking_thres = DECLARE_INT_ZH("tracker.tracking_threshold", 5,
        "确认跟踪所需的连续检测帧数", 0, 120);
    params.matcher_gate = DECLARE_DOUBLE_ZH("tracker.matcher_gate", 30.0,
        "机器人目标匹配的马氏距离门限", 0.0, 200.0, 0.001);
    params.outpost_matcher_gate = DECLARE_DOUBLE_ZH("tracker.outpost_matcher_gate", 0.4,
        "前哨站目标匹配的马氏距离门限", 0.0, 20.0, 0.001);

    rbp.sigma_a_xy = DECLARE_DOUBLE_ZH("tracker.sigma_a_xy", 10.0,
        "过程噪声：xy方向加速度", 0.0, 100.0, 0.001);
    rbp.sigma_a_z = DECLARE_DOUBLE_ZH("tracker.sigma_a_z", 1.0,
        "过程噪声：z方向加速度", 0.0, 10.0, 0.001);
    rbp.sigma_a_yaw = DECLARE_DOUBLE_ZH("tracker.sigma_a_yaw", 1.0,
        "过程噪声：yaw角加速度", 0.0, 40.0, 0.001);
    rbp.sigma_h = DECLARE_DOUBLE_ZH("tracker.sigma_h", 0.5,
        "过程噪声：装甲板高度漂移", 0.0, 5.0, 0.001);
    rbp.sigma_r0 = DECLARE_DOUBLE_ZH("tracker.sigma_r", 1.0,
        "过程噪声：装甲板半径漂移", 0.0, 5.0, 0.001);
    rbp.meas_dist_k = DECLARE_DOUBLE_ZH("tracker.measure_distance_k", 0.43,
        "测量噪声距离权重因子", 0.0, 5.0, 0.001);
    rbp.yaw_log_k = DECLARE_DOUBLE_ZH("tracker.yaw_log_k", 0.005,
        "yaw测量噪声对数因子", 0.0, 0.2, 0.0001);

    opp.sigma_a_yaw = DECLARE_DOUBLE_ZH("tracker.outpost.sigma_a_yaw", 1.0,
        "前哨站yaw加速度噪声", 0.0, 10.0, 0.001);
    opp.sigma_q_xy = DECLARE_DOUBLE_ZH("tracker.outpost.sigma_q_xy", 1.0,
        "前哨站xy漂移噪声", 0.0, 20.0, 0.001);
    opp.sigma_q_z = DECLARE_DOUBLE_ZH("tracker.outpost.sigma_q_z", 1.0,
        "前哨站z漂移噪声", 0.0, 10.0, 0.001);
    opp.meas_dist_k = DECLARE_DOUBLE_ZH("tracker.outpost.measure_distance_k", 0.43,
        "前哨站测量噪声距离权重因子", 0.0, 5.0, 0.001);
    opp.yaw_log_k = DECLARE_DOUBLE_ZH("tracker.outpost.yaw_log_k", 0.005,
        "前哨站yaw测量噪声对数因子", 0.0, 0.2, 0.0001);
    // clang-format on

#undef DECLARE_INT_ZH
#undef DECLARE_DOUBLE_ZH

    return params;
}

bool ArmorSolverNode::applyTrackerParamUpdate(
    const rclcpp::Parameter& param, Tracker::Params& params) {
    const auto& name = param.get_name();
    if (name == "tracker.lost_threshold") {
        params.lost_thres = param.as_double();
        return true;
    }
    if (name == "tracker.tracking_threshold") {
        params.tracking_thres = param.as_int();
        return true;
    }
    if (name == "tracker.matcher_gate") {
        params.matcher_gate = param.as_double();
        return true;
    }
    if (name == "tracker.outpost_matcher_gate") {
        params.outpost_matcher_gate = param.as_double();
        return true;
    }
    auto& rbp = params.robot_params;
    if (name == "tracker.sigma_a_xy") {
        rbp.sigma_a_xy = param.as_double();
        return true;
    }
    if (name == "tracker.sigma_a_z") {
        rbp.sigma_a_z = param.as_double();
        return true;
    }
    if (name == "tracker.sigma_a_yaw") {
        rbp.sigma_a_yaw = param.as_double();
        return true;
    }
    if (name == "tracker.sigma_h") {
        rbp.sigma_h = param.as_double();
        return true;
    }
    if (name == "tracker.sigma_r") {
        rbp.sigma_r0 = param.as_double();
        return true;
    }
    if (name == "tracker.measure_distance_k") {
        rbp.meas_dist_k = param.as_double();
        return true;
    }
    if (name == "tracker.yaw_log_k") {
        rbp.yaw_log_k = param.as_double();
        return true;
    }

    auto& opp = params.outpost_params;
    if (name == "tracker.outpost.sigma_a_yaw") {
        opp.sigma_a_yaw = param.as_double();
        return true;
    }
    if (name == "tracker.outpost.sigma_q_xy") {
        opp.sigma_q_xy = param.as_double();
        return true;
    }
    if (name == "tracker.outpost.sigma_q_z") {
        opp.sigma_q_z = param.as_double();
        return true;
    }
    if (name == "tracker.outpost.measure_distance_k") {
        opp.meas_dist_k = param.as_double();
        return true;
    }
    if (name == "tracker.outpost.yaw_log_k") {
        opp.yaw_log_k = param.as_double();
        return true;
    }
    return false;
}

rcl_interfaces::msg::SetParametersResult
    ArmorSolverNode::onSetParameters(const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool tracker_updated = false;
    {
        std::lock_guard<std::mutex> lock(tracker_mutex_);
        for (const auto& param : parameters) {
            tracker_updated = applyTrackerParamUpdate(param, tracker_params_) || tracker_updated;
        }
        if (tracker_updated) {
            tracker_.set_params(tracker_params_);
        }
    }
    return result;
}

void ArmorSolverNode::armorsCallback(rm_interfaces::msg::Armors::SharedPtr armors_msg) {
    // Tranform armor position from image frame to world coordinate
    for (auto& armor : armors_msg->armors) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = armors_msg->header;
        ps.pose   = armor.pose;
        try {
            armor.pose = tf2_buffer_->transform(ps, target_frame_, tf2::durationFromSec(0.001)).pose;
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "Transform error");
            return;
        }
    }
    armors_msg->header.frame_id = target_frame_;

    // Init message
    // Update tracker
    std::string number;
    const auto msg_stamp = armors_msg->header.stamp;
    rclcpp::Time msg_time(msg_stamp);
    if (last_time_.nanoseconds() == 0) {
        last_time_ = msg_time; // 初始化上一帧时间
    }
    double dts = (msg_time - last_time_).seconds();
    if (dts < 0.0) {
        RCLCPP_ERROR(get_logger(), "delta time period < 0");
        dts = 0.0;             // 避免时间戳回跳导致的负时间间隔
    }
    last_time_ = msg_time;
    rm_interfaces::msg::Target target_msg;
    rm_interfaces::msg::Measurement measurement_msg;
    bool publish_measurement = false;
    {
        std::lock_guard<std::mutex> lock(tracker_mutex_);
        if (tracker_.state != Tracker::IDLE) {
            tracker_.predict(dts);
        }
        if (tracker_.state == Tracker::IDLE) {
            number = tracker_.first_meet_u(*armors_msg);
        } else {
            tracker_.update(*armors_msg);
            const auto& z        = tracker_.get_measurement();
            measurement_msg      = rm_interfaces::msg::Measurement();
            measurement_msg.x1   = z[0];
            measurement_msg.y1   = z[1];
            measurement_msg.z1   = z[2];
            measurement_msg.yaw1 = z[3];
            measurement_msg.x2   = z[4];
            measurement_msg.y2   = z[5];
            measurement_msg.z2   = z[6];
            measurement_msg.yaw2 = z[7];
            publish_measurement  = true;
        }
        target_msg = tracker_.get_target();
    }
    if (publish_measurement) {
        measure_pub_->publish(measurement_msg);
    }

    target_msg.header.stamp    = msg_stamp;
    target_msg.header.frame_id = target_frame_;
    if (!number.empty()) {
        target_msg.id = number;
    }

    // Publish the target_msg
    target_pub_->publish(target_msg);

    // Publish visualization markers
    if (debug_mode_ && marker_pub_ != nullptr) {
        publishMarkers(target_msg);
    }
}

void ArmorSolverNode::publishMarkers(const rm_interfaces::msg::Target& target_msg) noexcept {
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;
    MarkerArray marry;
    int id = 0;

    const std_msgs::msg::Header hdr = target_msg.header;

    int a_n = target_msg.armors_num;
    if (a_n != 4 && a_n != 3) {
        // Not tracking, delete all markers
        position_marker_.action  = Marker::DELETE;
        armors_marker_.action    = Marker::DELETE;
        linear_v_marker_.action  = Marker::DELETE;
        angular_v_marker_.action = Marker::DELETE;

        marry.markers.emplace_back(linear_v_marker_);
        marry.markers.emplace_back(angular_v_marker_);
        marry.markers.emplace_back(position_marker_);
        marker_pub_->publish(marry);
        return;
    }

    if (target_msg.tracking) {
        linear_v_marker_.header  = hdr;
        angular_v_marker_.header = hdr;
        position_marker_.header  = hdr;

        position_marker_.action        = Marker::ADD;
        position_marker_.id            = id++;
        position_marker_.pose.position = target_msg.position;
        bool is_outpost                = target_msg.id == "outpost";
        if (is_outpost) {
            position_marker_.pose.position.z =
                (target_msg.position.z + target_msg.z1 + target_msg.z2) / 3;
        } else {
            position_marker_.pose.position.z = (target_msg.position.z + target_msg.z1) / 2;
        }

        linear_v_marker_.action = Marker::ADD;
        linear_v_marker_.id     = id++;
        linear_v_marker_.points.clear();
        linear_v_marker_.points.emplace_back(position_marker_.pose.position);
        geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
        arrow_end.x += target_msg.velocity.x;
        arrow_end.y += target_msg.velocity.y;
        arrow_end.z += target_msg.velocity.z;
        linear_v_marker_.points.emplace_back(arrow_end);
        angular_v_marker_.action = Marker::ADD;
        angular_v_marker_.id     = id++;
        angular_v_marker_.points.clear();
        angular_v_marker_.points.emplace_back(position_marker_.pose.position);
        arrow_end = position_marker_.pose.position;
        arrow_end.z += target_msg.v_yaw / M_PI;
        angular_v_marker_.points.emplace_back(arrow_end);

        armors_marker_.action  = Marker::ADD;
        armors_marker_.header  = hdr;
        armors_marker_.scale.y = target_msg.id == "1" || target_msg.id == "Bb" ? 0.23 : 0.135;
        geometry_msgs::msg::Point point_armor;
        for (int i = 0; i < a_n; i++) {
            double tmp_yaw = target_msg.yaw + i * (2 * M_PI / a_n);
            double radius;
            if (a_n == 4) {
                bool is_another_pair = (i == 1 || i == 3);
                radius               = is_another_pair ? target_msg.radius1 : target_msg.radius0;
                point_armor.z        = is_another_pair ? target_msg.z1 : target_msg.position.z;
            } else if (a_n == 3) {
                radius = OutpostModel::OUTPOST_RADIUS;
                if (i == 0) {
                    point_armor.z = target_msg.position.z;
                } else if (i == 1) {
                    point_armor.z = target_msg.z1;
                } else if (i == 2) {
                    point_armor.z = target_msg.z2;
                }
            } else {
                RCLCPP_ERROR(get_logger(), "Invalid armors num");
            }

            point_armor.x                = target_msg.position.x - radius * std::cos(tmp_yaw);
            point_armor.y                = target_msg.position.y - radius * std::sin(tmp_yaw);
            armors_marker_.id            = id++;
            armors_marker_.pose.position = point_armor;
            tf2::Quaternion q;
            q.setRPY(0, is_outpost ? -0.2618 : 0.2618, tmp_yaw);
            armors_marker_.pose.orientation = tf2::toMsg(q);
            marry.markers.emplace_back(armors_marker_);

            // ===== 在装甲板头顶加文字 =====
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = hdr;
            text_marker.ns     = "armor_id_text";
            text_marker.id     = 100 + id; // 确保和 armors_marker_.id 不重复
            text_marker.type   = Marker::TEXT_VIEW_FACING;
            text_marker.action = Marker::ADD;

            // 文字高度
            text_marker.scale.z = 0.2;

            // 颜色白色，透明一些
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 0.5;

            // 位置：和装甲板一样，但 z 往上抬一点
            text_marker.pose = armors_marker_.pose;
            text_marker.pose.position.z += 0.30;

            // 显示的文字内容：对应 i
            text_marker.text = std::to_string(i);

            marry.markers.emplace_back(text_marker);
        }
    } else {
        position_marker_.action  = Marker::DELETE;
        armors_marker_.action    = Marker::DELETE;
        linear_v_marker_.action  = Marker::DELETE;
        angular_v_marker_.action = Marker::DELETE;
    }
    marry.markers.emplace_back(linear_v_marker_);
    marry.markers.emplace_back(angular_v_marker_);
    marry.markers.emplace_back(position_marker_);

    marker_pub_->publish(marry);
}

} // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorSolverNode)
