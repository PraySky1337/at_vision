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
#include <memory>
#include <vector>
// project
#include "armor_solver/motion_model.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/heartbeat.hpp"

namespace fyt::auto_aim {
constexpr double MIN_DT = 0.002;
constexpr double MAX_DT = 0.1;
ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions& options)
    : Node("armor_solver", options)
    , dt_(0.004)
    , solver_(nullptr)
    , matcher_(get_logger()) {
    // Register logger
    FYT_REGISTER_LOGGER("armor_solver", "~/fyt2024-log", INFO);
    RCLCPP_INFO(get_logger(), "Starting ArmorSolver node");

    debug_mode_          = this->declare_parameter("debug", true);
    bool matcher_verbose = this->declare_parameter("matcher.verbose", false);

    // Tracker
    double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.2);
    double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
    matcher_.setVerbose(matcher_verbose);
    matcher_.setConstraints(max_match_distance, max_match_yaw_diff, true);
    tracker_                 = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
    tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
    lost_time_thres_         = this->declare_parameter("tracker.lost_time_thres", 0.3);

    initKF();
    // Callback groups
    armors_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscriber with tf2 message_filter
    tf2_buffer_          = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = armors_callback_group_;
    armors_sub_.subscribe(this, "armor_detector/armors", rmw_qos_profile_sensor_data, sub_options);
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

void ArmorSolverNode::initKF() {
    // EKF
    // xa = x_armor, xc = x_robot_center
    // state: xc, v_xc, yc, v_yc, zc, v_zc, yaw, v_yaw, r, d_zc
    // measurement: p, y, d, yaw
    // f - Process function
    auto f = Predict(0.005);
    // h - Observation function
    auto h = Measure();
    // update_Q - process noise covariance matrix
    s2qxyz_ = declare_parameter("kf.sigma2_q_x", 20.0);
    s2qyaw_ = declare_parameter("kf.sigma2_q_yaw", 100.0);
    s2qr_   = declare_parameter("kf.sigma2_q_r", 800.0);

    auto u_q = [this]() {
        Eigen::Matrix<double, X_N, X_N> q;
        const double t = std::max(dt_, 1e-3);
        double x = s2qxyz_, y = s2qxyz_, z = s2qxyz_, yaw = s2qyaw_, r = s2qr_;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
        double q_z_z = pow(t, 4) / 4 * z, q_z_vz = pow(t, 3) / 2 * x, q_vz_vz = pow(t, 2) * z;
        double q_yaw_yaw = pow(t, 4) / 4 * yaw, q_yaw_vyaw = pow(t, 3) / 2 * x,
               q_vyaw_vyaw = pow(t, 2) * yaw;
        double q_r         = pow(t, 4) / 4 * r;
        double q_l         = pow(t, 4) / 4;
        double q_h         = pow(t, 4) / 4;
        // clang-format off
    //    xc      v_xc    yc      v_yc    zc      v_zc    yaw         v_yaw       r       h      l
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,          0,          0,      0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,          0,          0,      0,      0,
          0,      0,      q_y_y,  q_y_vy, 0,      0,      0,          0,          0,      0,      0,
          0,      0,      q_y_vy, q_vy_vy,0,      0,      0,          0,          0,      0,      0,
          0,      0,      0,      0,      q_z_z,  q_z_vz, 0,          0,          0,      0,      0,
          0,      0,      0,      0,      q_z_vz, q_vz_vz,0,          0,          0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_yaw_yaw,  q_yaw_vyaw, 0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_yaw_vyaw, q_vyaw_vyaw,0,      0,      0,
          0,      0,      0,      0,      0,      0,      0,          0,          q_r,    0,      0,
          0,      0,      0,      0,      0,      0,      0,          0,          0,      q_l,    0,
          0,      0,      0,      0,      0,      0,      0,          0,          0,      0,      q_h;

        // clang-format on
        return q;
    };
    // update_R - measurement noise covariance matrix
    r_x_     = declare_parameter("kf.r_x", 0.05);
    r_y_     = declare_parameter("kf.r_y", 0.05);
    r_z_     = declare_parameter("kf.r_z", 0.05);
    r_yaw_   = declare_parameter("kf.r_yaw", 0.02);
    auto u_r = [this](const Eigen::Matrix<double, Z_N, 1>& z) {
        Eigen::Matrix<double, Z_N, Z_N> r;
        // clang-format off
    r << r_x_ * std::abs(z[0]), 0, 0, 0,
         0, r_y_ * std::abs(z[1]), 0, 0,
         0, 0, r_z_ * std::abs(z[2]), 0,
         0, 0, 0, r_yaw_ * std::sqrt(z[0]*z[0] + z[1]*z[1] + z[2]*z[2]);
        // clang-format on
        return r;
    };
    // P - error estimate covariance matrix
    Eigen::Matrix<double, X_N, X_N> P0 = Eigen::Matrix<double, X_N, X_N>::Identity();

    double alpha = declare_parameter("ukf.alpha", 0.1);
    double beta  = declare_parameter("ukf.beta", 2.0);
    double kappa = declare_parameter("ukf.kappa", 0.0);
    tracker_->kf = std::make_unique<RobotStateUKF>(f, h, u_q, u_r, P0, alpha, beta, kappa);
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

    // Init message
    rm_interfaces::msg::Measurement measure_msg;
    rm_interfaces::msg::Target target_msg;
    rclcpp::Time time          = armors_msg->header.stamp;
    target_msg.header.stamp    = time;
    target_msg.header.frame_id = target_frame_;
    if (last_time_.nanoseconds() > 0) {
        dt_ = (time - last_time_).seconds();
    }
    dt_ = std::clamp(dt_, MIN_DT, MAX_DT);

    // Update tracker
    bool publish_measure = false;
    TrackerSnapshot snapshot;
    if (tracker_->tracker_state == Tracker::LOST) {
        tracker_->init(armors_msg);
        target_msg.id       = tracker_->tracked_id;
        target_msg.tracking = false;
    } else {
        tracker_->lost_thres = std::abs(static_cast<int>(lost_time_thres_ / dt_));
        if (tracker_->tracked_id == "outpost") {
            tracker_->kf->setPredictFunc(Predict{dt_, MotionModel::CONSTANT_ROTATION});
        } else {
            tracker_->kf->setPredictFunc(Predict{dt_, MotionModel::CONSTANT_VEL_ROT});
        }
        tracker_->update(armors_msg, matcher_);
        // Publish measurement
        measure_msg.x1   = tracker_->measurement(0);
        measure_msg.y1   = tracker_->measurement(1);
        measure_msg.z1   = tracker_->measurement(2);
        measure_msg.yaw1 = tracker_->measurement(3);
        measure_msg.x2   = 0;
        measure_msg.y2   = 0;
        measure_msg.z2   = 0;
        measure_msg.yaw2 = 0;
        publish_measure  = true;

        if (tracker_->tracker_state == Tracker::DETECTING) {
            target_msg.id       = tracker_->tracked_id;
            target_msg.tracking = false;
        } else if (
            tracker_->tracker_state == Tracker::TRACKING
            || tracker_->tracker_state == Tracker::TEMP_LOST) {
            target_msg.tracking = true;
            // Fill target message
            auto header           = target_msg.header; // preserve header
            target_msg            = util::state2target(tracker_->target_state);
            target_msg.header     = header;
            target_msg.id         = tracker_->tracked_id;
            target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
        }
    }

    // Snapshot a safe copy for marker publishing (no Eigen inside)
    snapshot.target            = util::state2target(tracker_->target_state);
    snapshot.target.header     = target_msg.header;
    snapshot.target.tracking   = tracker_->tracker_state != Tracker::LOST;
    snapshot.target.id         = tracker_->tracked_id;
    snapshot.target.armors_num = static_cast<int>(tracker_->tracked_armors_num);
    snapshot.target.radius0    = tracker_->target_state[util::R];
    snapshot.target.l          = tracker_->target_state(util::L);
    snapshot.target.h          = tracker_->target_state(util::H);
    snapshot.target.z0         = snapshot.target.position.z;
    snapshot.valid             = tracker_->tracker_state != Tracker::LOST;

    // Store and Publish the target_msg
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        // Lazy initialize solver owing to weak_from_this() can't be called in constructor
        if (solver_ == nullptr) {
            solver_ = std::make_unique<Solver>(weak_from_this());
        }
        armor_target_ = target_msg;
        tracker_snapshot_ = snapshot;
        last_time_    = time;
    }
    if (publish_measure) {
        measure_pub_->publish(measure_msg);
    }
    target_pub_->publish(target_msg);
}

void ArmorSolverNode::publishMarkers(
    const rm_interfaces::msg::Target& target_msg,
    const rm_interfaces::msg::GimbalCmd& gimbal_cmd) noexcept {
    // 记录上一帧装甲板数量，用于 DELETE 多余的旧 marker
    RCLCPP_DEBUG(get_logger(), "Publish Marker");

    rm_interfaces::msg::Target display_target = target_msg;
    TrackerSnapshot snapshot_copy;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        snapshot_copy = tracker_snapshot_;
    }
    if (snapshot_copy.valid) {
        const auto& header = display_target.header;

        display_target          = snapshot_copy.target;
        display_target.header   = header;
        display_target.tracking = true;
    }

    // 公共 header
    position_marker_.header  = display_target.header;
    linear_v_marker_.header  = display_target.header;
    angular_v_marker_.header = display_target.header;
    armors_marker_.header    = display_target.header;
    selection_marker_.header = display_target.header;

    visualization_msgs::msg::MarkerArray marker_array;

    if (display_target.tracking || true) {
        // 位置球
        double xc = display_target.position.x;
        double yc = display_target.position.y;
        double zc = display_target.position.z;

        double vx = display_target.velocity.x;
        double vy = display_target.velocity.y;
        double vz = display_target.velocity.z;

        position_marker_.action          = visualization_msgs::msg::Marker::ADD;
        position_marker_.pose.position.x = xc;
        position_marker_.pose.position.y = yc;
        position_marker_.pose.position.z = (zc + display_target.h + zc) / 2.0;

        // 线速度箭头
        linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        linear_v_marker_.points.clear();
        linear_v_marker_.points.emplace_back(position_marker_.pose.position);
        geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
        arrow_end.x += vx;
        arrow_end.y += vy;
        arrow_end.z += vz;
        linear_v_marker_.points.emplace_back(arrow_end);

        // 角速度箭头
        angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        angular_v_marker_.points.clear();
        angular_v_marker_.points.emplace_back(position_marker_.pose.position);
        arrow_end = position_marker_.pose.position;
        arrow_end.z += display_target.v_yaw / M_PI;
        angular_v_marker_.points.emplace_back(arrow_end);

        // 装甲板 Marker
        armors_marker_.action = visualization_msgs::msg::Marker::ADD;
        if (snapshot_copy.valid && snapshot_copy.target.id == "1") {
            armors_marker_.scale.y = 0.23;
        } else {
            armors_marker_.scale.y = 0.135;
        }

        auto vec_xyza   = util::getRoboArmorPose(display_target);

        // 重新 ADD 当前帧装甲板及其 ID 文本
        int id = 0;
        for (const auto& armor : vec_xyza) {
            armors_marker_.id = id;
            auto& pos         = armors_marker_.pose.position;

            pos.x = armor.x();
            pos.y = armor.y();
            pos.z = armor.z();

            tf2::Quaternion q;
            q.setRPY(0, display_target.id == "outpost" ? -0.2618 : 0.2618, armor.w());
            armors_marker_.pose.orientation = tf2::toMsg(q);
            armors_marker_.action           = visualization_msgs::msg::Marker::ADD;
            marker_array.markers.emplace_back(armors_marker_);

            // Armor ID label above the cube
            armor_id_marker_.header = display_target.header;
            armor_id_marker_.id     = armors_marker_.id + 100; // 避免和 cube 冲突
            armor_id_marker_.pose   = armors_marker_.pose;
            armor_id_marker_.pose.position.z += 0.15;
            armor_id_marker_.text   = std::to_string(armors_marker_.id);
            armor_id_marker_.action = visualization_msgs::msg::Marker::ADD;
            marker_array.markers.emplace_back(armor_id_marker_);

            ++id;
        }

        // 选择点（弹道落点）
        selection_marker_.action = visualization_msgs::msg::Marker::ADD;
        selection_marker_.pose.position.y =
            gimbal_cmd.distance * std::sin(gimbal_cmd.yaw * M_PI / 180.0);
        selection_marker_.pose.position.x =
            gimbal_cmd.distance * std::cos(gimbal_cmd.yaw * M_PI / 180.0);
        selection_marker_.pose.position.z =
            gimbal_cmd.distance * std::sin(gimbal_cmd.pitch * M_PI / 180.0);

        // 轨迹点：注意顺序，先 clear 再填，再 ADD
        trajectory_marker_.header.frame_id = "muzzle_link";
        trajectory_marker_.action          = visualization_msgs::msg::Marker::ADD;
        trajectory_marker_.points.clear();
        for (const auto& point : solver_->getTrajectory()) {
            geometry_msgs::msg::Point p;
            p.x = point.first;
            p.y = 0.0; // 只用 x-z，y 设为 0
            p.z = point.second;
            trajectory_marker_.points.emplace_back(p);
        }

        if (gimbal_cmd.fire_advice) {
            trajectory_marker_.color.r = 0;
            trajectory_marker_.color.g = 1;
            trajectory_marker_.color.b = 0;
        } else {
            trajectory_marker_.color.r = 1;
            trajectory_marker_.color.g = 1;
            trajectory_marker_.color.b = 1;
        }

    } else {
        // 目标丢失：删除所有 marker，包括装甲板 cube + 文本 + 轨迹
        position_marker_.action   = visualization_msgs::msg::Marker::DELETE;
        linear_v_marker_.action   = visualization_msgs::msg::Marker::DELETE;
        angular_v_marker_.action  = visualization_msgs::msg::Marker::DELETE;
        armors_marker_.action     = visualization_msgs::msg::Marker::DELETE;
        selection_marker_.action  = visualization_msgs::msg::Marker::DELETE;
        trajectory_marker_.action = visualization_msgs::msg::Marker::DELETE;
        trajectory_marker_.points.clear();

        for (int i = 0; i < target_msg.armors_num; ++i) {
            armors_marker_.id = i;
            marker_array.markers.emplace_back(armors_marker_);

            armor_id_marker_.header = display_target.header;
            armor_id_marker_.id     = i + 100;
            armor_id_marker_.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.emplace_back(armor_id_marker_);
        }
    }

    // 公共 marker（装甲板和文本已经在上面 emplace_back 过了）
    marker_array.markers.emplace_back(position_marker_);
    marker_array.markers.emplace_back(trajectory_marker_);
    marker_array.markers.emplace_back(linear_v_marker_);
    marker_array.markers.emplace_back(angular_v_marker_);
    marker_array.markers.emplace_back(selection_marker_);

    marker_pub_->publish(marker_array);
}

} // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorSolverNode)
