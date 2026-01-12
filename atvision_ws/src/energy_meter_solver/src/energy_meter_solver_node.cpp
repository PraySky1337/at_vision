#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rm_interfaces/msg/energy_meter_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "energy_meter_solver/angle_fitter.hpp"
#include "energy_meter_solver/energy_meter_tracker.hpp"

namespace energy_meter {

class EnergyMeterSolverNode : public rclcpp::Node {
public:
    explicit EnergyMeterSolverNode(const rclcpp::NodeOptions& options)
        : Node("energy_meter_solver_node", options) {
        // 声明参数
        declare_parameter<std::string>("rune_type", "big");
        declare_parameter<double>("predict_time", 0.1);
        declare_parameter<std::string>("gimbal_frame", "gimbal");
        declare_parameter<std::string>("world_frame", "odom");
        declare_parameter<double>("smooth_alpha", 0.7);
        declare_parameter<int>("min_data_points", 5);
        declare_parameter<bool>("auto_detect_type", true);
        declare_parameter<double>("control_delay", 0.05);

        // 获取参数
        config_.rune_type       = get_parameter("rune_type").as_string();
        config_.predict_time    = get_parameter("predict_time").as_double();
        config_.gimbal_frame    = get_parameter("gimbal_frame").as_string();
        config_.world_frame     = get_parameter("world_frame").as_string();
        config_.smooth_alpha    = get_parameter("smooth_alpha").as_double();
        config_.min_data_points = get_parameter("min_data_points").as_int();

        config_.control_delay = get_parameter("control_delay").as_double();

        // 初始化组件
        angle_fitter_ = std::make_shared<AngleFitter>(config_);

        // 创建订阅
        marker_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "rune_detector/marker", rclcpp::SensorDataQoS(),
            std::bind(&EnergyMeterSolverNode::markerCallback, this, std::placeholders::_1));

        // 订阅弹道参数 (flying_time, prediction_delay) from rm_gimbal
        ballistic_params_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "rm_gimbal/ballistic_params", rclcpp::SensorDataQoS(),
            std::bind(
                &EnergyMeterSolverNode::ballisticParamsCallback, this, std::placeholders::_1));

        debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "energy_meter_solver/marker", rclcpp::SensorDataQoS());

        state_pub_ = create_publisher<rm_interfaces::msg::EnergyMeterState>(
            "energy_meter_solver/state", rclcpp::SensorDataQoS());
    }

private:
    // 配置
    EnergyMeterConfig config_;

    double predict_time_ = 0.0;
    double v_            = 0.0;
    // 组件
    std::shared_ptr<AngleFitter> angle_fitter_;

    // 能量机关追踪器
    Tracker tracker_;
    double last_update_timestamp_ = -1.0;

    // 订阅
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ballistic_params_sub_;

    // 弹道参数缓存 (from rm_gimbal)
    double cached_flying_time_{0.0};
    double cached_prediction_delay_{0.0};
    bool ballistic_params_received_{false};

    // 发布者
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
    rclcpp::Publisher<rm_interfaces::msg::EnergyMeterState>::SharedPtr state_pub_;

    // 状态
    bool is_tracking_                                    = false;
    rclcpp::Time last_tracking_timestamp_                = rclcpp::Time(0);
    static constexpr double TRACKING_LOSS_THRESHOLD_SEC_ = 0.5;

    static constexpr double RADIUS_THRESHOLD = 0.1;
    // 模型类型判断状态
    bool model_type_determined_           = false; // 是否已确定模型类型
    int detection_frame_count_            = 0;     // 检测帧计数
    static constexpr int DETECTION_FRAMES = 20;    // 前20帧用于判断类型

    // 防误判机制
    static constexpr double SWITCH_THRESHOLD    = 1.5; // v
    int switch_cooldown_count_                  = 0;   // 切换后的冷却帧计数
    static constexpr int SWITCH_COOLDOWN_FRAMES = 30;  // 切换后等待30帧再检测
    int frame_fps_                              = 0;
    static constexpr int FPS_THRESHOLD          = 100;

    // 速度统计（用于判断大小能量机关）
    int velocity_frame_count_                  = 0;
    double velocity_max_                       = -std::numeric_limits<double>::max();
    double velocity_min_                       = std::numeric_limits<double>::max();
    static constexpr int VELOCITY_SKIP_FRAMES_ = 5;

    void resetVelocityStats() {
        velocity_frame_count_ = 0;
        velocity_max_         = -std::numeric_limits<double>::max();
        velocity_min_         = std::numeric_limits<double>::max();
    }

    // 验证模型数据的辅助方法
    bool isValidModel(const AngleModel& model) const {
        if (model.is_big_rune) {
            return std::isfinite(model.sin_amplitude) && std::isfinite(model.sin_omega)
                && std::isfinite(model.sin_phase) && std::isfinite(model.sin_offset);
        } else {
            return std::isfinite(model.lin_omega) && std::isfinite(model.lin_offset);
        }
    }

    // 弹道参数回调 (from rm_gimbal)
    void ballisticParamsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            cached_flying_time_        = msg->data[0];
            cached_prediction_delay_   = msg->data[1];
            ballistic_params_received_ = true;
        }
    }

    void markerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        // 统一使用 energy_meter_tracker 处理大小能量机关
        // 预测时根据 model.is_big_rune 自动选择正弦波或线性模型

        // 检查消息是否为空
        if (msg->markers.empty()) {
            return;
        }

        // 从 MarkerArray 中提取 R 中心和所有目标
        Eigen::Vector3d r_center = Eigen::Vector3d::Zero();
        std::vector<Eigen::Vector3d> target_positions;
        std::vector<Eigen::Quaterniond> target_quats;

        bool found_rcenter = false;
        double timestamp   = 0.0;

        static constexpr uint32_t SPHERE_TYPE = 2;
        static constexpr uint32_t CUBE_TYPE   = 1;

        for (const auto& marker : msg->markers) {
            if (marker.action == visualization_msgs::msg::Marker::DELETE) {
                continue;
            }

            if (timestamp == 0.0) {
                timestamp = marker.header.stamp.sec + marker.header.stamp.nanosec / 1e9;
            }

            // SPHERE 是 R 中心
            if (marker.type == SPHERE_TYPE && !found_rcenter) {
                r_center = Eigen::Vector3d(
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
                found_rcenter = true;
            }

            // 所有 CUBE 是目标靶子（大能量机关可能有 1 或 2 个）
            if (marker.type == CUBE_TYPE) {
                target_positions.emplace_back(
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);

                const auto& q = marker.pose.orientation;
                target_quats.emplace_back(q.w, q.x, q.y, q.z);
            }
        }

        rclcpp::Time stamp = msg->markers[0].header.stamp;

        // 检查是否找到了必要的数据
        if (!found_rcenter || target_positions.empty()) {
            if (is_tracking_) {
                if (last_tracking_timestamp_.nanoseconds() == 0) {
                    last_tracking_timestamp_ = now();
                }
            }
            is_tracking_ = false;
            return;
        }

        // 恢复跟踪逻辑
        if (!is_tracking_ && last_tracking_timestamp_.nanoseconds() > 0) {
            double loss_duration_sec = (now() - last_tracking_timestamp_).seconds();
            if (loss_duration_sec >= TRACKING_LOSS_THRESHOLD_SEC_) {
                tracker_.state = Tracker::IDLE;
                RCLCPP_INFO(get_logger(), "目标丢失超过%.1fs", TRACKING_LOSS_THRESHOLD_SEC_);
            }
            last_tracking_timestamp_ = rclcpp::Time(0);
        }

        is_tracking_             = true;
        last_tracking_timestamp_ = rclcpp::Time(0);

        // 计算时间间隔
        double dt = (last_update_timestamp_ > 0) ? (timestamp - last_update_timestamp_) : 0.01;

        // 如果间隔超过 2s，重置所有状态
        if (dt > 2) {
            tracker_.state = Tracker::IDLE;
            tracker_.energy_ukf.reset();
            last_update_timestamp_ = -1.0;
            // 重置拟合器和类型判断状态
            angle_fitter_->reset();
            model_type_determined_ = false;
            detection_frame_count_ = 0;
            switch_cooldown_count_ = 0;
            resetVelocityStats();
            frame_fps_ = 0;
            RCLCPP_INFO(get_logger(), "时间间隔过大(%.2fs)，重置类型判断", dt);
            return;
        }

        last_update_timestamp_ = timestamp;

        // 第一次检测到目标时，初始化 tracker
        if (tracker_.state == Tracker::IDLE && !tracker_.energy_ukf.has_value()) {
            tracker_.first_meet_u(r_center, target_positions[0], target_quats[0]);
        }

        // 调用 tracker 进行预测和更新
        tracker_.predict(dt);
        tracker_.update(target_positions, target_quats);

        // 从 tracker 获取滤波后的状态，并进行参数拟合
        if (tracker_.energy_ukf.has_value()) {
            const auto& state        = tracker_.get_state();
            double filtered_velocity = state(V_ROLL);
            v_                       = state(V_ROLL);

            angle_fitter_->setFilteredVelocity(filtered_velocity);

            // 添加角度观测用于参数拟合（使用KF滤波后的角度和速度）
            AngleObservation obs;
            obs.timestamp        = timestamp;
            obs.continuous_angle = state(ROLL);
            obs.absolute_angle   = state(ROLL);
            obs.velocity         = filtered_velocity;
            obs.blade_offset     = tracker_.getCurrentBladeId();
            angle_fitter_->addObservation(obs);

            // 速度统计：从第5帧开始记录最大值和最小值
            velocity_frame_count_++;
            if (velocity_frame_count_ > VELOCITY_SKIP_FRAMES_) {
                double vel = filtered_velocity;

                if (vel > velocity_max_)
                    velocity_max_ = vel;
                if (vel < velocity_min_)
                    velocity_min_ = vel;
            }

            // 拟合逻辑
            if (angle_fitter_->getDataPointCount() >= config_.min_data_points) {

                if (!model_type_determined_) {
                    // 前20帧：判断模型类型
                    detection_frame_count_++;

                    angle_fitter_->fitAngleModelLSQ();
                    angle_fitter_->applyPosteriorConstraints();

                    // 通过速度变化范围判断大小符
                    // 小符: 速度恒定，max-min 很小
                    // 大符: 速度变化大，max-min 较大
                    double velocity_range                     = velocity_max_ - velocity_min_;
                    constexpr double VELOCITY_RANGE_THRESHOLD = 0.5; // 速度变化阈值
                    bool velocity_suggests_big_rune =
                        (std::abs(velocity_range) > VELOCITY_RANGE_THRESHOLD);

                    if (!velocity_suggests_big_rune) {
                        // 速度变化小，判定为小符
                        angle_fitter_->fitLinearModelPublic();
                    }

                    // 20帧后锁定类型
                    if (detection_frame_count_ >= DETECTION_FRAMES) {
                        model_type_determined_ = true;
                        auto final_model       = angle_fitter_->getModel();
                        RCLCPP_INFO(
                            get_logger(), "模型类型已确定: %s, 速度范围: %.3f (max=%.3f,min=%.3f)",
                            final_model.is_big_rune ? "大符" : "小符", velocity_range,
                            velocity_max_, velocity_min_);
                        // 重置速度统计
                        resetVelocityStats();
                        frame_fps_ = 0;
                    }

                } else {
                    // 类型已确定，只更新参数不切换类型
                    auto model = angle_fitter_->getModel();
                    if (model.is_big_rune) {
                        angle_fitter_->fitAngleModelLSQ();
                        angle_fitter_->applyPosteriorConstraints();
                    } else {
                        angle_fitter_->fitLinearModelPublic();
                    }

                    // 防误判机制：冷却期结束后，误差超过10 rad时切换类型
                    if (switch_cooldown_count_ > 0) {
                        switch_cooldown_count_--;
                    } else {
                        frame_fps_++;
                        if (frame_fps_ > FPS_THRESHOLD) {
                            double velocity_range = velocity_max_ - velocity_min_;
                            auto updated_model    = angle_fitter_->getModel();
                            if (velocity_range > SWITCH_THRESHOLD && !updated_model.is_big_rune) {
                                RCLCPP_WARN(
                                    get_logger(), "速度差值过大(%.2f )，切换类型: 小符 -> 大符",
                                    velocity_range);

                                angle_fitter_->fitAngleModelLSQ();
                                angle_fitter_->applyPosteriorConstraints();

                                // 启动冷却
                                switch_cooldown_count_ = SWITCH_COOLDOWN_FRAMES;

                            } else if (
                                velocity_range < SWITCH_THRESHOLD && updated_model.is_big_rune) {
                                RCLCPP_WARN(
                                    get_logger(), "速度差值过小(%.2f )，切换类型: 大符 ->小符 ",
                                    velocity_range);

                                angle_fitter_->fitLinearModelPublic();

                                // 启动冷却
                                switch_cooldown_count_ = SWITCH_COOLDOWN_FRAMES;
                            }
                            frame_fps_ = 0;
                            resetVelocityStats();
                        }
                    }
                }
            }
        }

        // 使用 tracker 状态生成预测结果
        if (tracker_.state == Tracker::TRACKING || tracker_.state == Tracker::TEMP_LOST) {
            const auto& state = tracker_.get_state();
            const auto& model = angle_fitter_->getModel();

            // 使用 rm_gimbal 发送的弹道参数，如果没有收到则使用配置值
            // flying_time = 弹丸飞行时间, prediction_delay = 云台响应延迟
            // 加上图像发布时间到当前时间的延迟
            double image_delay  = (now() - stamp).seconds();
            double predict_time = ballistic_params_received_
                                    ? (cached_flying_time_ + cached_prediction_delay_ + image_delay)
                                    : (config_.predict_time + config_.control_delay + image_delay);

            predict_time_ = predict_time;

            // 从实际观测计算当前靶的角度和半径
            Eigen::Vector3d rel_pos = target_positions[0] - r_center;
            double radius           = rel_pos.norm();
            if (radius < RADIUS_THRESHOLD)
                radius = 0.7;

            // 使用旋转矩阵获取旋转平面基向量，计算角度
            Eigen::Matrix3d rot           = target_quats[0].toRotationMatrix();
            Eigen::Vector3d u             = rot.col(2); // Z轴：θ=0 方向
            Eigen::Vector3d v             = rot.col(1); // Y轴：θ=π/2 方向
            double current_observed_angle = std::atan2(rel_pos.dot(v), rel_pos.dot(u));

            double predicted_roll_current;
            double predicted_roll_base;

            // 根据模型类型和有效性选择预测方法
            double v_now = state(V_ROLL);

            if (model.is_valid && model.is_big_rune) {
                // 大能量机关：正弦波模型
                double a     = model.sin_amplitude;
                double b     = model.sin_offset;
                double v_min = b - a;
                double v_max = b + a;

                // 检查速度是否在模型有效范围内: v ∈ [b-a, b+a]
                bool velocity_in_range = (v_now >= v_min - 0.1 * a && v_now <= v_max + 0.1 * a);

                if (velocity_in_range) {
                    // 使用拟合的三角函数模型预测
                    // v(t) = a·sin(ωt + φ) + b
                    // Δθ = ∫[0,Δt] v(τ) dτ = -a/ω·cos(φ + ω·Δt) + a/ω·cos(φ) + b·Δt
                    double omega = model.sin_omega;

                    // 从当前速度反推相位 φ_now
                    double sin_phi = (v_now - b) / a;
                    sin_phi        = std::clamp(sin_phi, -1.0, 1.0);
                    double phi_now = std::asin(sin_phi);

                    // 未来相位
                    double phi_future = phi_now + omega * predict_time;

                    // 角度增量 = -a/ω·cos(φ_future) + a/ω·cos(φ_now) + b·Δt
                    double delta_theta = -a / omega * std::cos(phi_future)
                                       + a / omega * std::cos(phi_now) + b * predict_time;

                    predicted_roll_current = current_observed_angle - delta_theta;
                    predicted_roll_base    = state(ROLL) - delta_theta;
                } else {
                    // 速度超出范围，回退到 KF 速度线性预测
                    predicted_roll_current = current_observed_angle - v_now * predict_time;
                    predicted_roll_base    = state(ROLL) - v_now * predict_time;
                }

            } else if (model.is_valid && !model.is_big_rune) {
                // 小能量机关：使用拟合的线性模型 (恒定角速度)
                double omega       = model.lin_omega; // 拟合得到的角速度
                double delta_theta = omega * predict_time;

                predicted_roll_current = current_observed_angle - delta_theta;
                predicted_roll_base    = state(ROLL) - delta_theta;

            } else {
                // 模型无效，回退到 KF 速度线性预测
                predicted_roll_current = current_observed_angle - v_now * predict_time;
                predicted_roll_base    = state(ROLL) - v_now * predict_time;
            }

            // 预测位置（绕R中心旋转，考虑旋转平面倾斜）
            Eigen::Vector3d predicted_position = r_center
                                               + radius * std::cos(predicted_roll_current) * u
                                               + radius * std::sin(predicted_roll_current) * v;

            int current_blade_id = tracker_.getCurrentBladeId();

            PredictionResult prediction;
            prediction.predicted_angle    = predicted_roll_base;
            prediction.predicted_position = predicted_position;
            prediction.is_valid           = true;

            AngleObservation angle_obs;
            angle_obs.timestamp        = timestamp;
            angle_obs.absolute_angle   = state(ROLL);
            angle_obs.continuous_angle = state(ROLL);
            angle_obs.blade_offset     = current_blade_id;

            ObservedFrame frame;
            frame.timestamp           = timestamp;
            frame.R_center_world      = r_center;
            frame.target_center_world = target_positions[0];
            frame.target_orientation  = target_quats[0];
            frame.is_valid            = true;

            publishDebugInfo(angle_obs, prediction, frame, stamp, msg);
        }
    }

    void publishDebugInfo(
        const AngleObservation& angle_obs, const PredictionResult& prediction,
        const ObservedFrame& frame, const rclcpp::Time& stamp,
        const visualization_msgs::msg::MarkerArray::SharedPtr raw_markers = nullptr) {
        try {
            // Validate publisher
            if (!state_pub_) {
                RCLCPP_ERROR(get_logger(), "Failed to publish state: publisher not initialized");
                return;
            }

            // Create state message
            auto state_msg = std::make_unique<rm_interfaces::msg::EnergyMeterState>();

            // Fill header
            state_msg->header.stamp    = stamp;
            state_msg->header.frame_id = config_.world_frame;

            // Angle information - validate before assignment
            if (!std::isfinite(angle_obs.absolute_angle)
                || !std::isfinite(angle_obs.continuous_angle)) {
                RCLCPP_ERROR(
                    get_logger(),
                    "❌ Cannot publish state: invalid angle observations - "
                    "abs=%.3f, cont=%.3f",
                    angle_obs.absolute_angle, angle_obs.continuous_angle);
                return;
            }

            state_msg->absolute_angle   = angle_obs.absolute_angle;
            state_msg->continuous_angle = angle_obs.continuous_angle;
            state_msg->blade_offset     = angle_obs.blade_offset;

            // Prediction - validate before assignment
            if (!std::isfinite(prediction.predicted_angle)) {
                RCLCPP_ERROR(
                    get_logger(), "❌ Cannot publish state: invalid predicted angle - %.3f",
                    prediction.predicted_angle);
                return;
            }

            state_msg->predicted_angle      = prediction.predicted_angle;
            state_msg->predicted_position.x = prediction.predicted_position.x();
            state_msg->predicted_position.y = prediction.predicted_position.y();
            state_msg->predicted_position.z = prediction.predicted_position.z();

            // 几何信息（供MPC使用）
            state_msg->r_center.x = frame.R_center_world.x();
            state_msg->r_center.y = frame.R_center_world.y();
            state_msg->r_center.z = frame.R_center_world.z();
            Eigen::Vector3d rel   = frame.target_center_world - frame.R_center_world;
            state_msg->radius     = rel.norm();

            // Model state
            const auto& model           = angle_fitter_->getModel();
            state_msg->model_valid      = model.is_valid;
            state_msg->is_big_rune      = model.is_big_rune;
            state_msg->fit_error        = model.fit_error;
            state_msg->data_points_used = model.data_points_used;

            // Model parameters - validate fit_error and data_points_used
            if (!std::isfinite(model.fit_error)) {
                RCLCPP_ERROR(
                    get_logger(), "❌ Cannot publish state: invalid fit_error - %.3f",
                    model.fit_error);
                return;
            }

            if (model.is_big_rune) {
                state_msg->sin_amplitude  = model.sin_amplitude;
                state_msg->sin_omega      = model.sin_omega;
                state_msg->sin_phase      = model.sin_phase;
                state_msg->sin_offset     = model.sin_offset;
                state_msg->sin_const_term = model.sin_const_term;
                state_msg->lin_omega      = 0.0;
                state_msg->lin_offset     = 0.0;
            } else {
                state_msg->sin_amplitude  = 0.0;
                state_msg->sin_omega      = 0.0;
                state_msg->sin_phase      = 0.0;
                state_msg->sin_offset     = 0.0;
                state_msg->sin_const_term = 0.0;
                state_msg->lin_omega      = model.lin_omega;
                state_msg->lin_offset     = model.lin_offset;
            }

            // 【新增】发布卡尔曼滤波的速度
            // state_msg->filtered_velocity =model.filtered_velocity
            state_msg->filtered_velocity = v_;

            state_msg->predicted_time = predict_time_;

            state_pub_->publish(std::move(state_msg));

            // 发布预测位置和所有靶子 Marker 可视化
            publishPredictionMarker(angle_obs, prediction, frame, stamp, raw_markers);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception while publishing state: %s", e.what());
        }
    }

    void publishPredictionMarker(
        const AngleObservation& angle_obs, const PredictionResult& prediction,
        const ObservedFrame& frame, rclcpp::Time stamp,
        const visualization_msgs::msg::MarkerArray::SharedPtr raw_markers = nullptr) {
        if (!debug_marker_pub_) {
            return;
        }

        auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

        // 获取目标半径和叶片角度间隔
        constexpr int BLADE_COUNT = 5;
        double target_radius      = (frame.target_center_world - frame.R_center_world).norm();
        if (target_radius < RADIUS_THRESHOLD)
            target_radius = 0.7;

        double blade_angle = 2.0 * M_PI / BLADE_COUNT; // 72° = 2π/5

        // 当前追踪靶子的实际位置（从观测直接获取）
        Eigen::Vector3d current_blade_pos = frame.target_center_world;

        // 使用旋转矩阵获取旋转平面基向量，计算角度
        Eigen::Matrix3d rot = frame.target_orientation.toRotationMatrix();
        Eigen::Vector3d u   = rot.col(2); // Z轴：θ=0 方向
        Eigen::Vector3d v   = rot.col(1); // Y轴：θ=π/2 方向

        Eigen::Vector3d rel_pos    = current_blade_pos - frame.R_center_world;
        double current_blade_angle = std::atan2(rel_pos.dot(v), rel_pos.dot(u));

        // 使用真实的靶ID（从angle_obs.blade_offset获取）
        int current_blade_id = angle_obs.blade_offset;

        // ========== 发布所有5个靶子的当前位置 ==========
        for (int i = 0; i < BLADE_COUNT; ++i) {
            Eigen::Vector3d position_world;

            bool is_current_target = (i == current_blade_id);

            if (is_current_target) {
                // 当前识别到的靶子 - 直接用实际观测位置
                position_world = current_blade_pos;
            } else {
                // 其他靶子 - 相对于当前靶进行旋转计算
                // 靶 i 相对于当前追踪靶的角度偏移
                int blade_diff = i - current_blade_id;
                double angle_i = current_blade_angle + blade_diff * blade_angle;

                // 使用旋转矩阵的列向量作为旋转平面基向量
                Eigen::Matrix3d rot = frame.target_orientation.toRotationMatrix();
                Eigen::Vector3d u   = rot.col(2); // Z轴：θ=0 方向
                Eigen::Vector3d v   = rot.col(1); // Y轴：θ=π/2 方向

                position_world = frame.R_center_world + target_radius * std::cos(angle_i) * u
                               + target_radius * std::sin(angle_i) * v;
            }

            // 创建靶子 Marker
            visualization_msgs::msg::Marker blade_marker;
            blade_marker.header.stamp    = stamp;
            blade_marker.header.frame_id = config_.world_frame;
            blade_marker.ns              = "blades";
            blade_marker.id              = i;
            blade_marker.type            = visualization_msgs::msg::Marker::CUBE;
            blade_marker.action          = visualization_msgs::msg::Marker::ADD;

            blade_marker.pose.position.x = position_world.x();
            blade_marker.pose.position.y = position_world.y();
            blade_marker.pose.position.z = position_world.z();
            // 设置正确的旋转方向（用于MPC计算旋转平面基向量）
            blade_marker.pose.orientation.w = frame.target_orientation.w();
            blade_marker.pose.orientation.x = frame.target_orientation.x();
            blade_marker.pose.orientation.y = frame.target_orientation.y();
            blade_marker.pose.orientation.z = frame.target_orientation.z();

            blade_marker.scale.x = 0.08;
            blade_marker.scale.y = 0.08;
            blade_marker.scale.z = 0.08;

            // 颜色区分
            if (is_current_target) {
                // 当前追踪的靶 - 红色（突出显示）
                blade_marker.color.r = 1.0f;
                blade_marker.color.g = 0.0f;
                blade_marker.color.b = 0.0f;
                blade_marker.color.a = 1.0f;
            } else {
                // 其他靶子 - 根据编号渐变颜色（蓝到绿）
                float ratio          = static_cast<float>(i) / (BLADE_COUNT - 1);
                blade_marker.color.r = 0.2f;
                blade_marker.color.g = 0.3f + 0.5f * ratio;
                blade_marker.color.b = 0.8f - 0.4f * ratio;
                blade_marker.color.a = 0.7f;
            }

            blade_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            marker_array->markers.push_back(blade_marker);

            // 创建序号文字 Marker
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.stamp    = stamp;
            text_marker.header.frame_id = config_.world_frame;
            text_marker.ns              = "blade_labels";
            text_marker.id              = i;
            text_marker.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action          = visualization_msgs::msg::Marker::ADD;

            // 文字位置稍微偏移
            text_marker.pose.position.x    = position_world.x();
            text_marker.pose.position.y    = position_world.y();
            text_marker.pose.position.z    = position_world.z() + 0.12;
            text_marker.pose.orientation.w = 1.0;

            text_marker.scale.z = 0.08;

            // 文字内容
            std::string label = std::to_string(i);
            if (is_current_target) {
                label += "*"; // 当前追踪的靶
            }

            text_marker.text = label;

            // 文字颜色 - 白色
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0f;

            text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            marker_array->markers.push_back(text_marker);
        }

        // ========== 发布预测位置 Marker ==========
        visualization_msgs::msg::Marker pred_marker;
        pred_marker.header.stamp    = stamp;
        pred_marker.header.frame_id = config_.world_frame;
        pred_marker.ns              = "prediction";
        pred_marker.id              = 0;
        pred_marker.type            = visualization_msgs::msg::Marker::CUBE;
        pred_marker.action          = visualization_msgs::msg::Marker::ADD;

        pred_marker.pose.position.x    = prediction.predicted_position.x();
        pred_marker.pose.position.y    = prediction.predicted_position.y();
        pred_marker.pose.position.z    = prediction.predicted_position.z();
        pred_marker.pose.orientation.w = 1.0;

        pred_marker.scale.x = 0.1;
        pred_marker.scale.y = 0.1;
        pred_marker.scale.z = 0.1;

        // 黄色表示预测位置
        pred_marker.color.r = 1.0f;
        pred_marker.color.g = 1.0f;
        pred_marker.color.b = 0.0f;
        pred_marker.color.a = 1.0f;

        pred_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_array->markers.push_back(pred_marker);

        // 预测位置文字标签
        visualization_msgs::msg::Marker pred_text;
        pred_text.header.stamp    = stamp;
        pred_text.header.frame_id = config_.world_frame;
        pred_text.ns              = "prediction_label";
        pred_text.id              = 0;
        pred_text.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        pred_text.action          = visualization_msgs::msg::Marker::ADD;

        pred_text.pose.position.x    = prediction.predicted_position.x();
        pred_text.pose.position.y    = prediction.predicted_position.y();
        pred_text.pose.position.z    = prediction.predicted_position.z() + 0.15;
        pred_text.pose.orientation.w = 1.0;

        pred_text.scale.z = 0.08;
        pred_text.text    = "PREDICT";

        pred_text.color.r = 1.0f;
        pred_text.color.g = 1.0f;
        pred_text.color.b = 0.0f;
        pred_text.color.a = 1.0f;

        pred_text.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_array->markers.push_back(pred_text);

        // ========== 发布R中心 Marker (SPHERE) ==========
        visualization_msgs::msg::Marker r_center_marker;
        r_center_marker.header.stamp    = stamp;
        r_center_marker.header.frame_id = config_.world_frame;
        r_center_marker.ns              = "r_center";
        r_center_marker.id              = 0;
        r_center_marker.type            = visualization_msgs::msg::Marker::SPHERE;
        r_center_marker.action          = visualization_msgs::msg::Marker::ADD;

        r_center_marker.pose.position.x    = frame.R_center_world.x();
        r_center_marker.pose.position.y    = frame.R_center_world.y();
        r_center_marker.pose.position.z    = frame.R_center_world.z();
        r_center_marker.pose.orientation.w = 1.0;

        r_center_marker.scale.x = 0.1;
        r_center_marker.scale.y = 0.1;
        r_center_marker.scale.z = 0.1;

        // 蓝色表示R中心
        r_center_marker.color.r = 0.0f;
        r_center_marker.color.g = 0.5f;
        r_center_marker.color.b = 1.0f;
        r_center_marker.color.a = 1.0f;

        r_center_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_array->markers.push_back(r_center_marker);

        debug_marker_pub_->publish(std::move(marker_array));
    }
};

} // namespace energy_meter

RCLCPP_COMPONENTS_REGISTER_NODE(energy_meter::EnergyMeterSolverNode)
