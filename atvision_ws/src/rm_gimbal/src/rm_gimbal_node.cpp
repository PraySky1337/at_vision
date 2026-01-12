#include "rm_gimbal/rm_gimbal_node.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <utility>

#include <Eigen/Dense>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/duration.hpp>

namespace {
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

namespace rm_gimbal {

GimbalNode::GimbalNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("gimbal_node", options)
    , device_(parser_)
    , tf_broadcaster_(*this)
    , aiming_color_(static_cast<int>(EnemyColor::UNKNOWN))
    , solver_(nullptr) {
    RCLCPP_INFO(get_logger(), "Starting GimbalNode with ballistic solver");

    // TF2 buffer and listener
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string target_component_name =
        declare_parameter("target_component_name", "armor_detector");
    if (declare_parameter("use_judgement_color", false)) {
        detector_client_ =
            std::make_shared<rclcpp::AsyncParametersClient>(this, target_component_name);
    } else {
        RCLCPP_INFO(
            get_logger(), "Gimbal Node will not set enemy color on armor detector, "
                          "because use_judgement_color is false.");
    }

    // 模拟器模式参数
    use_simulator_ = declare_parameter("use_simulator", false);

    // 仅非模拟器模式下连接下位机
    if (!use_simulator_) {
        this->init_parser();
        try {
            if (device_.open(0x0483)) {
                RCLCPP_INFO(get_logger(), "Gimbal Node already");
            } else {
                RCLCPP_FATAL(get_logger(), "Failed to open gimbal device");
            }
        } catch (...) {
            RCLCPP_FATAL(get_logger(), "Failed to open gimbal device");
            device_.open(0x0483);
        }
    } else {
        RCLCPP_INFO(get_logger(), "Simulator mode: skip USB device");
    }

    use_roll_ = declare_parameter("use_roll", false);
    debug_    = declare_parameter("debug", true);
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.description = "unit: ms";
    timestamp_offset_ms_   = this->declare_parameter("timestamp_offset", 0.1, param_desc); // s

    // Declare and initialize solver parameters
    solver_params_ = declare_solver_parameters();
    atomic_solver_params_.update(solver_params_);

    // Timer callback group (for 250Hz solve timer)
    timer_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscribe to Target from armor_solver (replaces GimbalCmd subscription)
    target_sub_ = create_subscription<rm_interfaces::msg::Target>(
        "armor_solver/target", rclcpp::SensorDataQoS(),
        std::bind(&GimbalNode::target_callback, this, std::placeholders::_1));

    // Subscribe to rune marker (energy mechanism predicted position)
    rune_marker_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "energy_meter_solver/marker", rclcpp::SensorDataQoS(),
        std::bind(&GimbalNode::rune_marker_callback, this, std::placeholders::_1));

    // Subscribe to energy meter state (for MPC)
    energy_state_sub_ = create_subscription<rm_interfaces::msg::EnergyMeterState>(
        "energy_meter_solver/state", rclcpp::SensorDataQoS(),
        std::bind(&GimbalNode::energy_state_callback, this, std::placeholders::_1));

    // Create 250Hz solve timer
    solve_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(4), std::bind(&GimbalNode::solve_timer_callback, this),
        timer_callback_group_);

    // Visualization publisher
    marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("rm_gimbal/marker", 10);

    // Debug publisher for GimbalCmd monitoring
    if (debug_) {
        gimbal_cmd_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>(
            "rm_gimbal/cmd", rclcpp::SensorDataQoS());
        plan_cmd_pub_ = this->create_publisher<rm_interfaces::msg::PlanGimbalCmd>(
            "rm_gimbal/plan_cmd", rclcpp::SensorDataQoS());
    }

    // Publisher for ballistic params to energy_meter_solver
    // data[0] = flying_time, data[1] = prediction_delay
    ballistic_params_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "rm_gimbal/ballistic_params", rclcpp::SensorDataQoS());

    on_set_params_cb_ = add_on_set_parameters_callback(
        std::bind(&GimbalNode::on_params, this, std::placeholders::_1));

    init_markers();

    // 仅非模拟器模式下启动 USB 线程
    if (!use_simulator_) {
        thread_ = std::thread([this] {
            running_ = true;
            while (running_) {
                device_.handle_events();
            }
        });
    }
}

GimbalNode::~GimbalNode() {
    // 仅非模拟器模式下停止 USB 设备
    if (!use_simulator_) {
        running_ = false;
        device_.stop();
        if (thread_.joinable()) {
            thread_.join();
        }
    }
}

SolverParams GimbalNode::declare_solver_parameters() {
    SolverParams params{};

#define DECLARE_DOUBLE_ZH(name, default_val, desc_zh, min, max, step) \
    this->declare_parameter(name, default_val, makeDoubleDescriptor(desc_zh, min, max, step))

    // clang-format off
    params.shooting_range_w = DECLARE_DOUBLE_ZH("solver.shooting_range_width", 0.135,
        "射击窗口宽度（米）", 0.0, 1.0, 0.001);
    params.shooting_range_h = DECLARE_DOUBLE_ZH("solver.shooting_range_height", 0.135,
        "射击窗口高度（米）", 0.0, 1.0, 0.001);
    params.max_tracking_v_yaw = DECLARE_DOUBLE_ZH("solver.max_tracking_v_yaw", 6.0,
        "跟踪装甲板模式的最大角速度阈值", 0.0, 60.0, 0.01);
    params.prediction_delay = DECLARE_DOUBLE_ZH("solver.prediction_delay", 0.0,
        "额外预测延迟（秒）", -1.0, 1.0, 0.001);
    params.side_angle = DECLARE_DOUBLE_ZH("solver.side_angle", 15.0,
        "侧向角度阈值（度）", 0.0, 90.0, 0.1);
    params.min_switching_v_yaw = DECLARE_DOUBLE_ZH("solver.min_switching_v_yaw", 1.0,
        "最小切换角速度", 0.0, 10.0, 0.01);
    params.transfer_thresh = static_cast<int>(this->declare_parameter("solver.transfer_thresh", 5));

    // Trajectory compensator parameters
    params.compensator_type = this->declare_parameter("solver.compensator_type", "resistance");
    params.trajectory.iteration_times = static_cast<int>(
        this->declare_parameter("solver.iteration_times", 20));
    params.trajectory.bullet_speed = DECLARE_DOUBLE_ZH("solver.bullet_speed", 20.0,
        "弹丸速度（米/秒）", 5.0, 50.0, 0.1);
    params.trajectory.gravity = DECLARE_DOUBLE_ZH("solver.gravity", 9.8,
        "重力加速度", 9.0, 10.0, 0.00001);
    params.trajectory.resistance = DECLARE_DOUBLE_ZH("solver.resistance", 0.001,
        "空气阻力系数", 0.0, 0.1, 0.0000001);

    // MPC轨迹规划参数
    params.mpc.enable = this->declare_parameter("solver.mpc.enable", true);
    params.mpc.half_horizon = static_cast<int>(this->declare_parameter("solver.mpc.half_horizon", 50));
    params.mpc.dt = DECLARE_DOUBLE_ZH("solver.mpc.dt", 0.01,
        "MPC时间步长（秒）", 0.001, 0.02, 0.0001);
    params.mpc.max_acc_yaw = DECLARE_DOUBLE_ZH("solver.mpc.max_acc_yaw", 30.0,
        "yaw最大加速度（rad/s²）- 关键参数，需匹配实际云台能力", 5.0, 200.0, 1.0);
    params.mpc.max_acc_pitch = DECLARE_DOUBLE_ZH("solver.mpc.max_acc_pitch", 30.0,
        "pitch最大加速度（rad/s²）- 关键参数，需匹配实际云台能力", 5.0, 200.0, 1.0);
    params.mpc.Q_pos = DECLARE_DOUBLE_ZH("solver.mpc.Q_pos", 100.0,
        "MPC位置跟踪权重", 1.0, 1000.0, 1.0);
    params.mpc.Q_vel = DECLARE_DOUBLE_ZH("solver.mpc.Q_vel", 1.0,
        "MPC速度跟踪权重", 0.1, 100.0, 0.1);
    params.mpc.R = DECLARE_DOUBLE_ZH("solver.mpc.R", 1.0,
        "MPC控制量权重（增大使轨迹更平滑）", 0.01, 100.0, 0.1);
    params.mpc.rho = DECLARE_DOUBLE_ZH("solver.mpc.rho", 1.0,
        "ADMM罚参数", 0.001, 10.0, 0.001);
    params.mpc.max_iter = static_cast<int>(this->declare_parameter("solver.mpc.max_iter", 10));
    params.mpc.abs_tol = DECLARE_DOUBLE_ZH("solver.mpc.abs_tol", 1e-3,
        "MPC收敛容差", 1e-6, 1e-1, 1e-6);
    params.mpc.side_angle = DECLARE_DOUBLE_ZH("solver.mpc.side_angle", 15.0,
        "MPC轨迹生成侧向角度（度）", 0.0, 45.0, 0.1);
    params.mpc.min_switching_v_yaw = DECLARE_DOUBLE_ZH("solver.mpc.min_switching_v_yaw", 1.0,
        "MPC轨迹生成最小切换角速度", 0.0, 10.0, 0.01);
    // clang-format on

#undef DECLARE_DOUBLE_ZH

    return params;
}

bool GimbalNode::apply_solver_param_update(const rclcpp::Parameter& param) {
    const auto& name = param.get_name();
    bool updated     = false;

    if (name == "solver.max_tracking_v_yaw") {
        atomic_solver_params_.max_tracking_v_yaw.store(
            param.as_double(), std::memory_order_relaxed);
        updated = true;
    } else if (name == "solver.prediction_delay") {
        atomic_solver_params_.prediction_delay.store(param.as_double(), std::memory_order_relaxed);
        updated = true;
    } else if (name == "solver.side_angle") {
        atomic_solver_params_.side_angle.store(param.as_double(), std::memory_order_relaxed);
        updated = true;
    } else if (name == "solver.min_switching_v_yaw") {
        atomic_solver_params_.min_switching_v_yaw.store(
            param.as_double(), std::memory_order_relaxed);
        updated = true;
    }
    // MPC参数热更新
    else if (name == "solver.mpc.enable") {
        atomic_solver_params_.mpc_enable.store(param.as_bool(), std::memory_order_relaxed);
        // 运行时启用MPC时，尝试初始化
        if (param.as_bool() && solver_ != nullptr) {
            if (!solver_->initializeMPC()) {
                RCLCPP_WARN(get_logger(), "Failed to initialize MPC on parameter change");
            } else {
                RCLCPP_INFO(get_logger(), "MPC initialized via parameter change");
            }
        }
        updated = true;
    } else if (name == "solver.mpc.max_acc_yaw") {
        atomic_solver_params_.mpc_max_acc_yaw.store(param.as_double(), std::memory_order_relaxed);
        if (solver_) {
            solver_params_.mpc.max_acc_yaw = param.as_double();
            solver_->updateMPCParams(solver_params_.mpc);
        }
        updated = true;
    } else if (name == "solver.mpc.max_acc_pitch") {
        atomic_solver_params_.mpc_max_acc_pitch.store(param.as_double(), std::memory_order_relaxed);
        if (solver_) {
            solver_params_.mpc.max_acc_pitch = param.as_double();
            solver_->updateMPCParams(solver_params_.mpc);
        }
        updated = true;
    }

    return updated;
}

void GimbalNode::set_params(const std::string& color) {
    if (detector_client_->wait_for_service()) {
        std::vector<rclcpp::Parameter> params;
        params.emplace_back("detect_color", color);
        auto result = detector_client_->set_parameters(params);
        try {
            if (result.get()[0].successful) {
                aiming_color_ = color == "red" ? static_cast<int>(EnemyColor::RED)
                                               : static_cast<int>(EnemyColor::BLUE);
                RCLCPP_INFO(get_logger(), "Set enemy color to %s successfully.", color.data());
            } else {
                RCLCPP_WARN(
                    get_logger(), "Failed to set enemy color to %s: %s", color.data(),
                    result.get()[0].reason.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
        }
    } else {
        RCLCPP_WARN(get_logger(), "Service not available, cannot set enemy color.");
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
                // Try to apply solver parameter update
                apply_solver_param_update(p);
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
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "Bullet speed: %.2f", d.bullet_speed);

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

    if (d.self_color != aiming_color_ && detector_client_ != nullptr) {
        set_params(aiming_color_ == 0 ? "red" : "blue"); // 保持原逻辑
    }
}

void GimbalNode::target_callback(rm_interfaces::msg::Target::ConstSharedPtr target_msg) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    // Lazy initialize solver
    if (solver_ == nullptr) {
        solver_ = std::make_unique<BallisticSolver>(solver_params_, atomic_solver_params_);
        // Initialize MPC if enabled
        if (solver_params_.mpc.enable) {
            if (solver_->initializeMPC()) {
                RCLCPP_INFO(get_logger(), "MPC trajectory planner initialized");
            } else {
                RCLCPP_WARN(get_logger(), "Failed to initialize MPC trajectory planner");
            }
        }
    }
    armor_target_ = *target_msg;
}

void GimbalNode::rune_marker_callback(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
    // Lazy initialize solver
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (solver_ == nullptr) {
            solver_ = std::make_unique<BallisticSolver>(solver_params_, atomic_solver_params_);
            if (solver_params_.mpc.enable) {
                solver_->initializeMPC();
            }
        }
    }

    // Extract R center (SPHERE marker) and current blade info
    Eigen::Vector3d r_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_blade_pos = Eigen::Vector3d::Zero();  // 当前靶位置（用于计算半径）
    Eigen::Quaterniond target_quat = Eigen::Quaterniond::Identity();
    std_msgs::msg::Header header;
    bool found_r_center = false;
    bool found_prediction = false;
    bool found_current_blade = false;

    static constexpr uint32_t SPHERE_TYPE = 2;
    static constexpr uint32_t CUBE_TYPE = 1;

    for (const auto& marker : msg->markers) {
        if (marker.action == visualization_msgs::msg::Marker::DELETE) {
            continue;
        }

        // R center (SPHERE)
        if (marker.type == SPHERE_TYPE && !found_r_center) {
            r_center = Eigen::Vector3d(
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
            found_r_center = true;
        }

        // Prediction marker
        if (marker.ns == "prediction") {
            target_pos = Eigen::Vector3d(
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
            found_prediction = true;
            header = marker.header;  // 使用prediction marker的header
        }

        // Current blade (ns="blades") for orientation and position
        // 使用当前追踪靶（红色，id可能不是0）
        if (marker.ns == "blades" && marker.type == CUBE_TYPE) {
            // 检查是否是当前追踪的靶（红色: r=1, g=0, b=0）
            if (marker.color.r > 0.9 && marker.color.g < 0.1 && marker.color.b < 0.1) {
                current_blade_pos = Eigen::Vector3d(
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
                const auto& q = marker.pose.orientation;
                target_quat = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
                found_current_blade = true;
            }
        }
    }

    if (!found_prediction || !found_r_center || !found_current_blade) {
        return;
    }

    // Get angular velocity, r_center, and radius from cached energy state
    double angular_velocity = 0.0;
    double radius = 0.7;  // 默认半径
    {
        std::lock_guard<std::mutex> lock(energy_state_mutex_);
        if (energy_state_valid_) {
            angular_velocity = cached_energy_state_.filtered_velocity;
            // 使用energy_meter_solver计算的准确半径
            if (cached_energy_state_.radius > 0.1) {
                radius = cached_energy_state_.radius;
            }
            // 使用energy_meter_solver的R中心（更准确）
            r_center = Eigen::Vector3d(
                cached_energy_state_.r_center.x,
                cached_energy_state_.r_center.y,
                cached_energy_state_.r_center.z);
        }
    }

    // Calculate rotation plane basis vectors from blade orientation
    Eigen::Matrix3d rot = target_quat.toRotationMatrix();
    Eigen::Vector3d rotation_u = rot.col(2);  // Z轴：θ=0 方向
    Eigen::Vector3d rotation_v = rot.col(1);  // Y轴：θ=π/2 方向

    try {
        auto cmd = solver_->solveRune(
            target_pos, r_center, radius,
            rotation_u, rotation_v, angular_velocity,
            header, tf_buffer_);
        send_to_device(cmd);

        // Publish ballistic params to energy_meter_solver
        std_msgs::msg::Float64MultiArray ballistic_params;
        ballistic_params.data.push_back(solver_->getLastFlyingTime());
        ballistic_params.data.push_back(solver_->getLastPredictionDelay());
        ballistic_params_pub_->publish(ballistic_params);

        // Publish debug info
        if (debug_ && gimbal_cmd_pub_ != nullptr) {
            gimbal_cmd_pub_->publish(cmd);
        }

        // Publish PlanGimbalCmd for debug monitoring
        if (debug_ && plan_cmd_pub_ != nullptr) {
            rm_interfaces::msg::PlanGimbalCmd plan_cmd;
            plan_cmd.header = header;

            plan_cmd.raw_yaw   = static_cast<float>(cmd.yaw * M_PI / 180.0);
            plan_cmd.raw_pitch = static_cast<float>(cmd.pitch * M_PI / 180.0);

            // Get MPC feedforward
            auto ff = solver_->getFeedforward();
            plan_cmd.mpc_valid = ff.valid;
            if (ff.valid) {
                plan_cmd.ref_yaw       = plan_cmd.raw_yaw;
                plan_cmd.ref_yaw_vel   = static_cast<float>(ff.yaw_vel);
                plan_cmd.ref_yaw_acc   = static_cast<float>(ff.yaw_acc);
                plan_cmd.ref_pitch     = plan_cmd.raw_pitch;
                plan_cmd.ref_pitch_vel = static_cast<float>(ff.pitch_vel);
                plan_cmd.ref_pitch_acc = static_cast<float>(ff.pitch_acc);
            }

            // Get current gimbal state from TF
            try {
                auto gimbal_tf = tf_buffer_->lookupTransform(
                    header.frame_id, "gimbal_link", tf2::TimePointZero);
                tf2::Quaternion q;
                tf2::fromMsg(gimbal_tf.transform.rotation, q);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                plan_cmd.cur_yaw   = static_cast<float>(yaw);
                plan_cmd.cur_pitch = static_cast<float>(-pitch);
            } catch (const tf2::TransformException&) {
                plan_cmd.cur_yaw   = 0.0f;
                plan_cmd.cur_pitch = 0.0f;
            }

            plan_cmd.fire_advice = cmd.fire_advice;
            plan_cmd.distance    = static_cast<float>(cmd.distance);

            plan_cmd_pub_->publish(plan_cmd);
        }

        // Publish visualization markers for rune
        if (debug_ && marker_pub_ != nullptr) {
            visualization_msgs::msg::MarkerArray marker_array;

            // Target position marker
            visualization_msgs::msg::Marker target_marker;
            target_marker.header             = header;
            target_marker.ns                 = "rune_target";
            target_marker.id                 = 0;
            target_marker.type               = visualization_msgs::msg::Marker::SPHERE;
            target_marker.action             = visualization_msgs::msg::Marker::ADD;
            target_marker.pose.position.x    = target_pos.x();
            target_marker.pose.position.y    = target_pos.y();
            target_marker.pose.position.z    = target_pos.z();
            target_marker.pose.orientation.w = 1.0;
            target_marker.scale.x            = 0.1;
            target_marker.scale.y            = 0.1;
            target_marker.scale.z            = 0.1;
            target_marker.color.r            = 1.0f;
            target_marker.color.g            = 0.5f;
            target_marker.color.a            = 1.0f;
            target_marker.lifetime           = rclcpp::Duration::from_seconds(0.1);
            marker_array.markers.push_back(target_marker);

            marker_pub_->publish(marker_array);
        }
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000, "Rune TF error: %s", ex.what());
    }
}

void GimbalNode::energy_state_callback(rm_interfaces::msg::EnergyMeterState::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(energy_state_mutex_);
    cached_energy_state_ = *msg;
    energy_state_valid_ = true;
}

void GimbalNode::solve_timer_callback() {
    rm_interfaces::msg::Target target_snapshot;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (solver_ == nullptr) {
            return;
        }
        target_snapshot = armor_target_;
    }

    // If target never detected
    if (target_snapshot.header.frame_id.empty()) {
        send_empty_command();
        return;
    }

    rm_interfaces::msg::GimbalCmd gimbal_cmd;

    if (target_snapshot.tracking) {
        try {
            gimbal_cmd = solver_->solve(target_snapshot, now(), tf_buffer_);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF error: %s", ex.what());
            send_empty_command();
            return;
        }
    } else {
        gimbal_cmd.yaw_diff    = 0;
        gimbal_cmd.pitch_diff  = 0;
        gimbal_cmd.distance    = -1;
        gimbal_cmd.fire_advice = false;
    }

    // Send directly to device via USB (no ROS msg)
    send_to_device(gimbal_cmd);

    // Publish GimbalCmd for debug monitoring
    if (debug_ && gimbal_cmd_pub_ != nullptr) {
        gimbal_cmd_pub_->publish(gimbal_cmd);
    }

    // Publish PlanGimbalCmd for MPC debug monitoring
    if (debug_ && plan_cmd_pub_ != nullptr && solver_ != nullptr) {
        rm_interfaces::msg::PlanGimbalCmd plan_cmd;
        plan_cmd.header = target_snapshot.header;

        // 原始弹道解算结果（弧度）
        plan_cmd.raw_yaw   = static_cast<float>(gimbal_cmd.yaw * M_PI / 180.0);
        plan_cmd.raw_pitch = static_cast<float>(gimbal_cmd.pitch * M_PI / 180.0);

        // MPC前馈
        auto ff            = solver_->getFeedforward();
        plan_cmd.mpc_valid = ff.valid;
        if (ff.valid) {
            plan_cmd.ref_yaw       = static_cast<float>(plan_cmd.raw_yaw); // MPC目标位置
            plan_cmd.ref_yaw_vel   = static_cast<float>(ff.yaw_vel);
            plan_cmd.ref_yaw_acc   = static_cast<float>(ff.yaw_acc);
            plan_cmd.ref_pitch     = static_cast<float>(plan_cmd.raw_pitch);
            plan_cmd.ref_pitch_vel = static_cast<float>(ff.pitch_vel);
            plan_cmd.ref_pitch_acc = static_cast<float>(ff.pitch_acc);
        }

        // 当前云台状态（从TF获取）
        try {
            auto gimbal_tf = tf_buffer_->lookupTransform(
                target_snapshot.header.frame_id, "gimbal_link", tf2::TimePointZero);
            tf2::Quaternion q;
            tf2::fromMsg(gimbal_tf.transform.rotation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            plan_cmd.cur_yaw   = static_cast<float>(yaw);
            plan_cmd.cur_pitch = static_cast<float>(-pitch);
        } catch (const tf2::TransformException&) {
            plan_cmd.cur_yaw   = 0.0f;
            plan_cmd.cur_pitch = 0.0f;
        }
        plan_cmd.cur_yaw_vel   = 0.0f; // TODO: 从IMU获取
        plan_cmd.cur_pitch_vel = 0.0f;

        plan_cmd.fire_advice = gimbal_cmd.fire_advice;
        plan_cmd.distance    = static_cast<float>(gimbal_cmd.distance);

        plan_cmd_pub_->publish(plan_cmd);
    }

    // Publish visualization markers
    if (debug_ && marker_pub_ != nullptr) {
        publish_markers(target_snapshot, gimbal_cmd);
    }
}

void GimbalNode::send_to_device(const rm_interfaces::msg::GimbalCmd& cmd) {
    // 模拟器模式下不发送数据
    if (use_simulator_) {
        return;
    }

    SendVisionData vision_data;
    vision_data.header.id         = 0x02;
    vision_data.header.len        = sizeof(decltype(vision_data.data));
    vision_data.header.sof        = HeaderFrame::SoF();
    vision_data.eof               = HeaderFrame::EoF();
    vision_data.data.fire_advice  = cmd.fire_advice;
    vision_data.data.distance     = static_cast<float>(cmd.distance);
    vision_data.data.target_pitch = -static_cast<float>(cmd.pitch);
    vision_data.data.target_yaw   = static_cast<float>(cmd.yaw);

    // 填充MPC前馈量
    // 注意：pitch方向与下位机约定相反，需要取反（历史遗留问题）
    if (solver_ != nullptr) {
        auto ff = solver_->getFeedforward();
        if (ff.valid) {
            vision_data.data.ref_yaw_v   = static_cast<float>(ff.yaw_vel);
            vision_data.data.ref_pitch_v = -static_cast<float>(ff.pitch_vel);
            vision_data.data.ref_yaw_a   = static_cast<float>(ff.yaw_acc);
            vision_data.data.ref_pitch_a = -static_cast<float>(ff.pitch_acc);
        } else {
            vision_data.data.ref_yaw_v   = 0.0f;
            vision_data.data.ref_pitch_v = 0.0f;
            vision_data.data.ref_yaw_a   = 0.0f;
            vision_data.data.ref_pitch_a = 0.0f;
        }
    } else {
        vision_data.data.ref_yaw_v   = 0.0f;
        vision_data.data.ref_pitch_v = 0.0f;
        vision_data.data.ref_yaw_a   = 0.0f;
        vision_data.data.ref_pitch_a = 0.0f;
    }

    std::memcpy(buffer_, &vision_data, sizeof(SendVisionData));
    if (!device_.send_data(buffer_, sizeof(SendVisionData))) {
        RCLCPP_WARN(get_logger(), "Failed to send data");
    }
}

void GimbalNode::send_empty_command() {
    rm_interfaces::msg::GimbalCmd empty_cmd;
    empty_cmd.yaw_diff    = 0;
    empty_cmd.pitch_diff  = 0;
    empty_cmd.distance    = -1;
    empty_cmd.pitch       = 0;
    empty_cmd.yaw         = 0;
    empty_cmd.fire_advice = false;
    send_to_device(empty_cmd);
}

void GimbalNode::init_markers() noexcept {
    selection_marker_.ns      = "selection";
    selection_marker_.type    = visualization_msgs::msg::Marker::SPHERE;
    selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.1;
    selection_marker_.color.a                                                         = 0.3;
    selection_marker_.color.g                                                         = 1.0;
    selection_marker_.color.r                                                         = 1.0;

    predicted_marker_.ns      = "predicted_position";
    predicted_marker_.type    = visualization_msgs::msg::Marker::SPHERE;
    predicted_marker_.scale.x = predicted_marker_.scale.y = predicted_marker_.scale.z = 0.08;
    predicted_marker_.color.a                                                         = 0.6;
    predicted_marker_.color.r                                                         = 1.0;
    predicted_marker_.color.g                                                         = 0.5;
    predicted_marker_.color.b                                                         = 0.2;

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

void GimbalNode::publish_markers(
    const rm_interfaces::msg::Target& target_msg,
    const rm_interfaces::msg::GimbalCmd& gimbal_cmd) noexcept {
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;
    MarkerArray marry;
    int id = 0;

    const std_msgs::msg::Header hdr = target_msg.header;

    if (target_msg.tracking) {
        selection_marker_.header  = hdr;
        trajectory_marker_.header = hdr;

        // Selection marker - shows where we're aiming
        selection_marker_.action = Marker::ADD;
        selection_marker_.id     = id++;
        selection_marker_.points.clear();
        selection_marker_.pose.position.y = gimbal_cmd.distance * sin(gimbal_cmd.yaw * M_PI / 180);
        selection_marker_.pose.position.x = gimbal_cmd.distance * cos(gimbal_cmd.yaw * M_PI / 180);
        selection_marker_.pose.position.z =
            gimbal_cmd.distance * sin(gimbal_cmd.pitch * M_PI / 180);

        // Predicted position marker
        predicted_marker_.header = hdr;
        predicted_marker_.id     = id++;
        predicted_marker_.action = Marker::DELETE;
        if (solver_ != nullptr) {
            const auto predicted_position = solver_->getPredictedPosition();
            if (predicted_position.has_value()) {
                predicted_marker_.action             = Marker::ADD;
                predicted_marker_.pose.position.x    = predicted_position->x();
                predicted_marker_.pose.position.y    = predicted_position->y();
                predicted_marker_.pose.position.z    = predicted_position->z();
                predicted_marker_.pose.orientation.x = 0.0;
                predicted_marker_.pose.orientation.y = 0.0;
                predicted_marker_.pose.orientation.z = 0.0;
                predicted_marker_.pose.orientation.w = 1.0;
            }
        }

        // Trajectory marker
        trajectory_marker_.action = Marker::ADD;
        trajectory_marker_.id     = id++;
        trajectory_marker_.points.clear();
        trajectory_marker_.header.frame_id = "muzzle_link";
        if (solver_ != nullptr) {
            for (const auto& point : solver_->getTrajectory()) {
                geometry_msgs::msg::Point p;
                p.x = point.first;
                p.z = point.second;
                trajectory_marker_.points.emplace_back(p);
            }
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
        selection_marker_.action  = Marker::DELETE;
        trajectory_marker_.action = Marker::DELETE;
        predicted_marker_.action  = Marker::DELETE;
        predicted_marker_.header  = hdr;
    }

    marry.markers.emplace_back(selection_marker_);
    marry.markers.emplace_back(predicted_marker_);
    marry.markers.emplace_back(trajectory_marker_);

    marker_pub_->publish(marry);
}

} // namespace rm_gimbal

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_gimbal::GimbalNode)
