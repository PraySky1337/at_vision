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
#include <vector>
// project
#include "rm_utils/common.hpp"
#include "rm_utils/heartbeat.hpp"

namespace fyt::auto_aim {

ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions& options)
    : Node("armor_solver", options)
    , dt_(0.004)
    , solver_(nullptr) {
    // Register logger
    FYT_REGISTER_LOGGER("armor_solver", "~/fyt2024-log", INFO);
    RCLCPP_INFO(get_logger(), "Starting ArmorSolver node");

    debug_mode_ = this->declare_parameter("debug", true);

    // Callback groups
    timer_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriber with tf2 message_filter
    tf2_buffer_          = std::make_shared<tf2_ros::Buffer>(this->get_clock());
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
    gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>(
        "armor_solver/cmd_gimbal", rclcpp::SensorDataQoS());
    marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_solver/marker", 10);
    // Timer 250 Hz
    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&ArmorSolverNode::timerCallback, this),
        timer_callback_group_);

    initMarkers();
}

void ArmorSolverNode::timerCallback() {
    rm_interfaces::msg::Target target_snapshot;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (solver_ == nullptr) {
            return;
        }
        target_snapshot = armor_target_;
    }

    // Init message
    rm_interfaces::msg::GimbalCmd control_msg;

    // If target never detected
    if (target_snapshot.header.frame_id.empty()) {
        control_msg.yaw_diff    = 0;
        control_msg.pitch_diff  = 0;
        control_msg.distance    = -1;
        control_msg.pitch       = 0;
        control_msg.yaw         = 0;
        control_msg.fire_advice = false;
        gimbal_pub_->publish(control_msg);
        return;
    }

    if (target_snapshot.tracking) {
        try {
            control_msg = solver_->solve(target_snapshot, this->now(), tf2_buffer_);
        } catch (...) {
            FYT_ERROR("armor_solver", "Something went wrong in solver!");
            control_msg.yaw_diff    = 0;
            control_msg.pitch_diff  = 0;
            control_msg.distance    = -1;
            control_msg.fire_advice = false;
        }
    } else {
        control_msg.yaw_diff    = 0;
        control_msg.pitch_diff  = 0;
        control_msg.distance    = -1;
        control_msg.fire_advice = false;
    }

    gimbal_pub_->publish(control_msg);

    if (debug_mode_ && marker_pub_ != nullptr) {
        publishMarkers(target_snapshot, control_msg);
    }
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
    armor_id_marker_.ns       = "armor_id";
    armor_id_marker_.type     = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    armor_id_marker_.scale.z  = 0.08;
    armor_id_marker_.color.a  = 1.0;
    armor_id_marker_.color.r  = 1.0;
    armor_id_marker_.color.g  = 1.0;
    armor_id_marker_.color.b  = 1.0;
    armors_marker_.ns         = "filtered_armors";
    armors_marker_.type       = visualization_msgs::msg::Marker::CUBE;
    armors_marker_.scale.x    = 0.03;
    armors_marker_.scale.z    = 0.125;
    armors_marker_.color.a    = 0.3;
    armors_marker_.color.b    = 1.0;
    selection_marker_.ns      = "selection";
    selection_marker_.type    = visualization_msgs::msg::Marker::SPHERE;
    selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.1;

    selection_marker_.color.a  = 0.3;
    selection_marker_.color.g  = 1.0;
    selection_marker_.color.r  = 1.0;
    trajectory_marker_.ns      = "trajectory";
    trajectory_marker_.type    = visualization_msgs::msg::Marker::POINTS;
    trajectory_marker_.scale.x = 0.01;
    trajectory_marker_.scale.y = 0.01;
    trajectory_marker_.color.a = 0.3;
    trajectory_marker_.color.r = 1.0;
    trajectory_marker_.color.g = 0.75;
    trajectory_marker_.color.b = 0.79;
    trajectory_marker_.points.clear();
}

void ArmorSolverNode::armorsCallback(rm_interfaces::msg::Armors::SharedPtr armors_msg) {
    // Tranform armor position from image frame to world coordinate
    for (auto& armor : armors_msg->armors) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = armors_msg->header;
        ps.pose   = armor.pose;
        try {
            armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
        } catch (const tf2::TransformException& ex) {
            FYT_ERROR("armor_solver", "Transform error: {}", ex.what());
            return;
        }
    }
    armors_msg->header.frame_id = target_frame_;

    // Init message
    // Update tracker
    std::string number;
    auto n     = now();
    double dts = (n - armors_msg->header.stamp).seconds();
    if (target.state != armor_tracker::Tracker::IDLE) {
        target.predict(dts);
    }
    if (target.state == armor_tracker::Tracker::IDLE) {
        number = target.first_meet_u(*armors_msg);
    } else {
        target.update(*armors_msg);
    }
    rm_interfaces::msg::Target target_msg = target.get_target();

    target_msg.header.stamp    = n;
    target_msg.header.frame_id = target_frame_;
    target_msg.id              = number;
    // Store and Publish the target_msg
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        // Lazy initialize solver owing to weak_from_this() can't be called in constructor
        if (solver_ == nullptr) {
            solver_ = std::make_unique<Solver>(weak_from_this());
        }
        armor_target_ = target_msg;
    }
    target_pub_->publish(target_msg);
}

void ArmorSolverNode::publishMarkers(
    const rm_interfaces::msg::Target& target_msg,
    const rm_interfaces::msg::GimbalCmd& gimbal_cmd) noexcept {
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;
    MarkerArray marry;
    const std_msgs::msg::Header hdr = target_msg.header;

    int a_n = target_msg.armors_num;
    assert(a_n == 4 || a_n == 3);
    if (target_msg.tracking) {

        linear_v_marker_.header  = hdr;
        angular_v_marker_.header = hdr;
        position_marker_.header  = hdr;

        position_marker_.action          = visualization_msgs::msg::Marker::ADD;
        position_marker_.pose.position   = target_msg.position;
        position_marker_.pose.position.z = (target_msg.position.z + target_msg.z1) / 2;

        linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        linear_v_marker_.points.clear();
        linear_v_marker_.points.emplace_back(position_marker_.pose.position);
        geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
        arrow_end.x += target_msg.velocity.x;
        arrow_end.y += target_msg.velocity.y;
        arrow_end.z += target_msg.velocity.z;
        linear_v_marker_.points.emplace_back(arrow_end);
        angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        angular_v_marker_.points.clear();
        angular_v_marker_.points.emplace_back(position_marker_.pose.position);
        arrow_end = position_marker_.pose.position;
        arrow_end.z += target_msg.v_yaw / M_PI;
        angular_v_marker_.points.emplace_back(arrow_end);

        armors_marker_.action  = visualization_msgs::msg::Marker::ADD;
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
                // TODO: 3 装甲逻辑
            } else {
                RCLCPP_ERROR(get_logger(), "Invalid armors num");
            }

            point_armor.x                = target_msg.position.x - radius * std::cos(tmp_yaw);
            point_armor.y                = target_msg.position.y - radius * std::sin(tmp_yaw);
            armors_marker_.id            = i;
            armors_marker_.pose.position = point_armor;
            tf2::Quaternion q;
            q.setRPY(0, target_msg.id == "outpost" ? -0.2618 : 0.2618, tmp_yaw);
            armors_marker_.pose.orientation = tf2::toMsg(q);
            marry.markers.emplace_back(armors_marker_);

            // ===== 在装甲板头顶加文字 =====
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = hdr;
            text_marker.ns     = "armor_id_text";
            text_marker.id     = 100 + i; // 确保和 armors_marker_.id 不重复
            text_marker.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;

            // 文字高度
            text_marker.scale.z = 0.2; // 根据实际需求调

            // 颜色（白色、不透明）
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;

            // 位置：和装甲板一样，但 z 往上抬一点
            text_marker.pose = armors_marker_.pose;
            text_marker.pose.position.z += 0.15; // 抬高一点避免和模型重叠

            // 显示的文字内容：对应 i
            text_marker.text = std::to_string(i);

            marry.markers.emplace_back(text_marker);
        }
    } else {
        position_marker_.action  = Marker::DELETE;
        armors_marker_.action    = Marker::DELETE;
        linear_v_marker_.action  = Marker::DELETE;
        selection_marker_.action = Marker::DELETE;
        angular_v_marker_.action = Marker::DELETE;
    }
    marry.markers.emplace_back(angular_v_marker_);
    marry.markers.emplace_back(position_marker_);

    geometry_msgs::msg::Point p0, p1;
    p0 = target_msg.position;
    marker_pub_->publish(marry);
}

void ArmorSolverNode::publish_cv_3d_markers(const rm_interfaces::msg::Target& target) noexcept {
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;

    MarkerArray marr;
    marr.markers.reserve(3);

    // 通用 header
    std_msgs::msg::Header hdr = target.header;
    hdr.frame_id              = "odom";

    // ========== 位置：球 ==========
    Marker m_pos;
    m_pos.header = hdr;
    m_pos.ns     = "cv3d";
    m_pos.id     = 0;
    m_pos.type   = Marker::SPHERE;
    m_pos.action = Marker::ADD;

    m_pos.pose.position      = target.position;
    m_pos.pose.orientation.w = 1.0;

    m_pos.scale.x = 0.12;
    m_pos.scale.y = 0.12;
    m_pos.scale.z = 0.12;

    m_pos.color.r = 0.1f;
    m_pos.color.g = 0.9f;
    m_pos.color.b = 0.1f;
    m_pos.color.a = 0.3f;

    // ========== 速度：箭头 ==========
    Marker m_vel;
    m_vel.header = hdr;
    m_vel.ns     = "cv3d";
    m_vel.id     = 1;
    m_vel.type   = Marker::ARROW;
    m_vel.action = Marker::ADD;

    geometry_msgs::msg::Point p0, p1;
    p0 = target.position;

    auto& vel   = target.velocity;
    double norm = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);

    // 速度方向 & 长度
    double len = 0.0;
    if (norm > 1e-6) {
        // 可按需缩放，以免太长/太短
        double scale_len = 0.2; // 自己调
        len              = norm * scale_len;

        p1.x = p0.x + vel.x / norm * len;
        p1.y = p0.y + vel.y / norm * len;
        p1.z = p0.z + vel.z / norm * len;
    } else {
        // 速度很小时就画个几乎看不见的短箭头
        p1 = p0;
    }

    m_vel.points.clear();
    m_vel.points.push_back(p0);
    m_vel.points.push_back(p1);

    // 箭头粗细
    m_vel.scale.x = 0.03;                      // shaft 直径
    m_vel.scale.y = 0.06;                      // head 直径
    m_vel.scale.z = std::max(0.05, len * 0.3); // head 长度，给个最小值

    m_vel.color.r = 1.0f;
    m_vel.color.g = 0.1f;
    m_vel.color.b = 0.1f;
    m_vel.color.a = 0.95f;

    // ========== yaw 速度：箭头（示例：竖直向上） ==========
    Marker m_yaw_vel;
    m_yaw_vel.header = hdr;
    m_yaw_vel.ns     = "cv3d";
    m_yaw_vel.id     = 2;
    m_yaw_vel.type   = Marker::ARROW;
    m_yaw_vel.action = Marker::ADD;

    geometry_msgs::msg::Point yaw_p0, yaw_p1;
    yaw_p0 = target.position;
    yaw_p1 = yaw_p0;

    // 用 v_yaw 控制 z 方向长度，按需缩放
    double yaw_len = target.v_yaw * 0.2; // 自己调比例
    yaw_p1.z += yaw_len;

    m_yaw_vel.points.clear();
    m_yaw_vel.points.push_back(yaw_p0);
    m_yaw_vel.points.push_back(yaw_p1);

    m_yaw_vel.scale.x = 0.02;
    m_yaw_vel.scale.y = 0.04;
    m_yaw_vel.scale.z = std::max(0.03, std::abs(yaw_len) * 0.4);

    m_yaw_vel.color.r = 0.1f;
    m_yaw_vel.color.g = 0.1f;
    m_yaw_vel.color.b = 1.0f;
    m_yaw_vel.color.a = 0.9f;

    marr.markers.push_back(std::move(m_pos));
    marr.markers.push_back(std::move(m_vel));
    marr.markers.push_back(std::move(m_yaw_vel));

    if (marker_pub_) {
        marker_pub_->publish(marr);
    }
}

} // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorSolverNode)
