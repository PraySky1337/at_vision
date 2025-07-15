// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "armor_tracker/tracker_node.hpp"

// STD
#include "armor_tracker/extended_kalman_filter.hpp"
#include <cmath>
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/subscription_options.hpp>
#include <vector>

namespace rm_auto_aim {
ArmorTrackerNode::ArmorTrackerNode(
    const rclcpp::Node* detector_node, const std::string& name, const std::string& ns,
    const rclcpp::NodeOptions& options)
    : Node(name, ns, options)
    , detector_node_name_with_ns(detector_node->get_fully_qualified_name()) {
    RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");
    declareParameters();
    initTrackers();
    initEkf();
    initServices();
    initTf();
    initSubscribers();
    initPublishers();
    initMarkers();
}
void ArmorTrackerNode::declareParameters() {
    debug               = declare_parameter("debug", false);
    max_armor_distance_ = declare_parameter("max_armor_distance", 10.0);

    //  EKF 动态噪声、测量噪声
    s2qxyz_max_  = declare_parameter("ekf.sigma2_q_xyz_max", 0.1);
    s2qxyz_min_  = declare_parameter("ekf.sigma2_q_xyz_min", 0.05);
    s2qyaw_max_  = declare_parameter("ekf.sigma2_q_yaw_max", 10.0);
    s2qyaw_min_  = declare_parameter("ekf.sigma2_q_yaw_min", 5.0);
    s2qr_        = declare_parameter("ekf.sigma2_q_r", 80.0);
    r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05);
    r_yaw        = declare_parameter("ekf.r_yaw", 0.02);

    //  左右 gimbal 目标坐标系
    target_frame_ = declare_parameter("target_frame", "odom");
}

void ArmorTrackerNode::initTrackers() {
    double max_match_distance = declare_parameter("tracker.max_match_distance", 0.15);
    double max_match_yaw_diff = declare_parameter("tracker.max_match_yaw_diff", 1.0);
    int tracking_thres        = declare_parameter("tracker.tracking_thres", 5);
    lost_time_thres_          = declare_parameter("tracker.lost_time_thres", 0.3);

    tracker_                 = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
    tracker_->tracking_thres = tracking_thres;
}

void ArmorTrackerNode::initEkf() {
    auto& trk = tracker_;
    /* ---------- ① 过程模型 f(x) & J_f(x) ---------- */
    auto f = [this](const Eigen::VectorXd& x) {
        Eigen::VectorXd xn = x;
        xn(0) += x(1) * dt_;      // xc += v_xc·dt
        xn(2) += x(3) * dt_;      // yc += v_yc·dt
        xn(4) += x(5) * dt_;      // za += v_za·dt
        xn(6) += x(7) * dt_;      // yaw += v_yaw·dt
        return xn;
    };

    auto j_f = [this](const Eigen::VectorXd&) {
        Eigen::MatrixXd jf = Eigen::MatrixXd::Identity(9, 9);
        jf(0, 1)           = dt_; // ∂xc/∂v_xc
        jf(2, 3)           = dt_; // ∂yc/∂v_yc
        jf(4, 5)           = dt_; // ∂za/∂v_za
        jf(6, 7)           = dt_; // ∂yaw/∂v_yaw
        return jf;
    };

    /* ---------- ② 观测模型 h(x) & J_h(x) ---------- */
    auto h = [](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(4);
        const double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
        z << xc - r * std::sin(yaw), // xa
            yc + r * std::cos(yaw),  // ya
            x(4),                    // za
            yaw;                     // yaw
        return z;
    };

    auto j_h = [](const Eigen::VectorXd& x) {
        Eigen::MatrixXd jh(4, 9);
        jh.setZero();
        const double yaw = x(6), r = x(8);
        jh(0, 0) = 1.0;
        jh(0, 6) = -r * std::cos(yaw);
        jh(0, 8) = -std::sin(yaw);

        jh(1, 2) = 1.0;
        jh(1, 6) = -r * std::sin(yaw);
        jh(1, 8) = std::cos(yaw);

        jh(2, 4) = 1.0;
        jh(3, 6) = 1.0;
        return jh;
    };

    /* ---------- ③ 过程噪声 Q(k) ---------- */
    auto u_q = [this](const Eigen::VectorXd& xp) {
        const double vx = xp(1), vy = xp(3), v_yaw = xp(7);
        const double dx = std::hypot(vx, vy);
        const double dy = std::abs(v_yaw);

        const double q_xyz = std::exp(-dy) * (s2qxyz_max_ - s2qxyz_min_) + s2qxyz_min_;
        const double q_yaw = std::exp(-dx) * (s2qyaw_max_ - s2qyaw_min_) + s2qyaw_min_;

        Eigen::MatrixXd q(9, 9);
        q.setZero();
        auto fill = [&](int p, int v, double s2) {
            q(p, p) = std::pow(dt_, 4) / 4 * s2;
            q(p, v) = q(v, p) = std::pow(dt_, 3) / 2 * s2;
            q(v, v)           = std::pow(dt_, 2) * s2;
        };
        fill(0, 1, q_xyz); // x
        fill(2, 3, q_xyz); // y
        fill(4, 5, q_xyz); // z
        fill(6, 7, q_yaw); // yaw
        q(8, 8) = std::pow(dt_, 4) / 4 * s2qr_;
        return q;
    };

    /* ---------- ④ 测量噪声 R(k) ---------- */
    auto u_r = [this](const Eigen::VectorXd& z) {
        Eigen::DiagonalMatrix<double, 4> r;
        const double xyz_s2 = r_xyz_factor;
        r.diagonal() << std::abs(xyz_s2 * z(0)), std::abs(xyz_s2 * z(1)), std::abs(xyz_s2 * z(2)),
            r_yaw;
        return r;
    };

    /* ---------- ⑤ EKF 实例化 ---------- */
    Eigen::DiagonalMatrix<double, 9> p0;
    p0.setIdentity();
    trk->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
}

void ArmorTrackerNode::initServices() {
    using Trigger = std_srvs::srv::Trigger;
    using Req     = Trigger::Request::SharedPtr;
    using Resp    = Trigger::Response::SharedPtr;

    reset_tracker_srv_ = create_service<Trigger>("~/reset", [this](Req, Resp res) {
        tracker_->tracker_state = Tracker::LOST;
        res->success            = true;
        RCLCPP_INFO(get_logger(), "Tracker reset!");
    });

    change_target_srv_ = create_service<Trigger>("~/change", [this](Req, Resp res) {
        tracker_->tracker_state = Tracker::CHANGE_TARGET;
        res->success            = true;
        RCLCPP_INFO(get_logger(), "change target!");
    });
}

void ArmorTrackerNode::initTf() {
    cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 共有计时器接口
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
}

void ArmorTrackerNode::initSubscribers() {
    armors_sub_.subscribe(
        this, detector_node_name_with_ns + "/armors", rmw_qos_profile_sensor_data);

    tf2_filter_ = std::make_shared<tf2_filter>(
        armors_sub_, *tf2_buffer_, target_frame_, 10, get_node_logging_interface(),
        get_node_clock_interface(), std::chrono::duration<int>(1));

    tf2_filter_->registerCallback([this](auto msg) {
        armorsCallback(std::const_pointer_cast<auto_aim_interfaces::msg::Armors>(msg));
    });
}

void ArmorTrackerNode::initPublishers() {
    info_pub_ = create_publisher<auto_aim_interfaces::msg::TrackerInfo>("~/info", 10);

    target_pub_ =
        create_publisher<auto_aim_interfaces::msg::Target>("~/target", rclcpp::SensorDataQoS());
}

void ArmorTrackerNode::initMarkers() {
    // 统一的模板 marker
    visualization_msgs::msg::Marker sphere, arrow_v, arrow_w, cube;

    sphere.ns      = "position";
    sphere.type    = visualization_msgs::msg::Marker::SPHERE;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.1;
    sphere.color.a                                   = 1.0;
    sphere.color.g                                   = 1.0;

    arrow_v.ns      = "linear_v";
    arrow_v.type    = visualization_msgs::msg::Marker::ARROW;
    arrow_v.scale.x = 0.03;
    arrow_v.scale.y = 0.05;
    arrow_v.color.a = 1.0;
    arrow_v.color.r = arrow_v.color.g = 1.0;

    arrow_w.ns      = "angular_v";
    arrow_w.type    = visualization_msgs::msg::Marker::ARROW;
    arrow_w.scale.x = 0.03;
    arrow_w.scale.y = 0.05;
    arrow_w.color.a = 1.0;
    arrow_w.color.b = arrow_w.color.g = 1.0;

    cube.ns      = "armors";
    cube.type    = visualization_msgs::msg::Marker::CUBE;
    cube.scale.x = 0.03;
    cube.scale.z = 0.125;
    cube.color.a = 1.0;
    cube.color.r = 1.0;

    position_marker_  = sphere;
    linear_v_marker_  = arrow_v;
    angular_v_marker_ = arrow_w;
    armor_marker_     = cube;

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 10);
}

void ArmorTrackerNode::armorsCallback(
    const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg) {
    // Tranform armor position from image frame to world coordinate
    for (auto& armor : armors_msg->armors) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = armors_msg->header;
        ps.pose   = armor.pose;
        try {
            armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
        } catch (const tf2::ExtrapolationException& ex) {
            RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
            return;
        }
    }

    // Filter abnormal armors
    armors_msg->armors.erase(
        std::remove_if(
            armors_msg->armors.begin(), armors_msg->armors.end(),
            [this](const auto_aim_interfaces::msg::Armor& armor) {
                return abs(armor.pose.position.z) > 1.2
                    || Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm()
                           > max_armor_distance_;
            }),
        armors_msg->armors.end());

    // Init message
    auto_aim_interfaces::msg::TrackerInfo info_msg;
    auto_aim_interfaces::msg::Target target_msg;
    rclcpp::Time time          = armors_msg->header.stamp;
    target_msg.header.stamp    = time;
    target_msg.header.frame_id = target_frame_;

    // Update tracker
    if (tracker_->tracker_state == Tracker::LOST) {
        tracker_->init(armors_msg);
        target_msg.tracking = false;
    } else {
        dt_                  = (time - last_time_).seconds();
        tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);
        tracker_->update(armors_msg);

        // Publish Info
        info_msg.position_diff = tracker_->info_position_diff;
        info_msg.yaw_diff      = tracker_->info_yaw_diff;
        info_msg.position.x    = tracker_->measurement(0);
        info_msg.position.y    = tracker_->measurement(1);
        info_msg.position.z    = tracker_->measurement(2);
        info_msg.yaw           = tracker_->measurement(3);
        info_pub_->publish(info_msg);

        if (tracker_->tracker_state == Tracker::DETECTING) {
            target_msg.tracking = false;
        } else if (
            tracker_->tracker_state == Tracker::TRACKING
            || tracker_->tracker_state == Tracker::TEMP_LOST) {
            target_msg.tracking = true;
            // Fill target message
            const auto& state     = tracker_->target_state;
            target_msg.id         = tracker_->tracked_id;
            target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
            target_msg.position.x = state(0);
            target_msg.velocity.x = state(1);
            target_msg.position.y = state(2);
            target_msg.velocity.y = state(3);
            target_msg.position.z = state(4);
            target_msg.velocity.z = state(5);
            target_msg.yaw        = state(6);
            target_msg.v_yaw      = state(7);
            target_msg.radius_1   = state(8);
            target_msg.radius_2   = tracker_->another_r;
            target_msg.dz         = tracker_->dz;
        }
    }

    last_time_ = time;

    target_pub_->publish(target_msg);

    publishMarkers(target_msg);
}

void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target& target_msg) {
    position_marker_.header  = target_msg.header;
    linear_v_marker_.header  = target_msg.header;
    angular_v_marker_.header = target_msg.header;
    armor_marker_.header     = target_msg.header;

    visualization_msgs::msg::MarkerArray marker_array;
    if (target_msg.tracking) {
        double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
        double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
        double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
        double dz = target_msg.dz;

        position_marker_.action          = visualization_msgs::msg::Marker::ADD;
        position_marker_.pose.position.x = xc;
        position_marker_.pose.position.y = yc;
        position_marker_.pose.position.z = za + dz / 2;

        linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        linear_v_marker_.points.clear();
        linear_v_marker_.points.emplace_back(position_marker_.pose.position);
        geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
        arrow_end.x += vx;
        arrow_end.y += vy;
        arrow_end.z += vz;
        linear_v_marker_.points.emplace_back(arrow_end);

        angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
        angular_v_marker_.points.clear();
        angular_v_marker_.points.emplace_back(position_marker_.pose.position);
        arrow_end = position_marker_.pose.position;
        arrow_end.z += target_msg.v_yaw / M_PI;
        angular_v_marker_.points.emplace_back(arrow_end);

        armor_marker_.action  = visualization_msgs::msg::Marker::ADD;
        armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
        bool is_current_pair  = true;
        size_t a_n            = target_msg.armors_num;
        geometry_msgs::msg::Point p_a;
        double r = 0;
        for (size_t i = 0; i < a_n; i++) {
            double tmp_yaw = yaw + i * (2 * M_PI / a_n);
            // Only 4 armors has 2 radius and height
            if (a_n == 4) {
                r               = is_current_pair ? r1 : r2;
                p_a.z           = za + (is_current_pair ? 0 : dz);
                is_current_pair = !is_current_pair;
            } else {
                r     = r1;
                p_a.z = za;
            }
            p_a.x = xc - r * cos(tmp_yaw);
            p_a.y = yc - r * sin(tmp_yaw);

            armor_marker_.id            = i;
            armor_marker_.pose.position = p_a;
            tf2::Quaternion q;
            q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
            armor_marker_.pose.orientation = tf2::toMsg(q);
            marker_array.markers.emplace_back(armor_marker_);
        }
    } else {
        position_marker_.action  = visualization_msgs::msg::Marker::DELETEALL;
        linear_v_marker_.action  = visualization_msgs::msg::Marker::DELETEALL;
        angular_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;

        armor_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.emplace_back(armor_marker_);
    }

    marker_array.markers.emplace_back(position_marker_);
    marker_array.markers.emplace_back(linear_v_marker_);
    marker_array.markers.emplace_back(angular_v_marker_);
    marker_pub_->publish(marker_array);
}

} // namespace rm_auto_aim