#ifndef ENERGY_METER_SOLVER_TYPES_HPP_
#define ENERGY_METER_SOLVER_TYPES_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace energy_meter {

// ============================================================================
// 状态枚举
// ============================================================================

enum EnergyState : uint8_t {
    XC,
    YC,
    ZC,
    ROLL,   // 0 号装甲板的 ROLL
    V_ROLL, // 角速度
    STATE_MAX,
};

enum Measure : uint8_t { ARMOR_X, ARMOR_Y, ARMOR_Z, ARMOR_YAW, MEASURE_MAX };

// ============================================================================
// 工具函数
// ============================================================================

// 归一化到 (-π, π] 弧度
inline double normalize_rad(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a <= 0.0)
        a += 2.0 * M_PI;
    return a - M_PI;
}

// 计算最小角度差 to - from，结果在 (-π, π] 弧度
inline double shortest_rad(double from, double to) { return normalize_rad(to - from); }

// 根据上一帧角度 prev，使当前测量 raw 连续化（弧度版本）
inline double unwrap_rad(double prev, double raw) {
    double d = shortest_rad(prev, raw);
    return prev + d;
}

// xyz 转 yaw-pitch-distance
inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) {
    double x = xyz.x();
    double y = xyz.y();
    double z = xyz.z();

    double distance = xyz.norm();
    double yaw      = std::atan2(y, x);
    double pitch    = std::atan2(z, std::sqrt(x * x + y * y));

    return Eigen::Vector3d(yaw, pitch, distance);
}

inline double orientation2roll(const geometry_msgs::msg::Quaternion& q_msg) {
    tf2::Quaternion q;
    tf2::fromMsg(q_msg, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return roll;
}

// Eigen::Quaterniond 版本的重载
inline double orientation2roll(const Eigen::Quaterniond& q_eigen) {
    tf2::Quaternion q(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return roll;
}

// ============================================================================
// 能量机关求解器数据类型
// ============================================================================

/**
 * @brief 能量机关检测器的单帧观测数据
 * 包含R标记中心位置和目标装甲板方向
 */
struct ObservedFrame {
    double timestamp;                      // Unix时间戳(秒)
    Eigen::Vector3d R_center_world;        // R标记中心在世界坐标系中的位置
    Eigen::Vector3d target_center_world;   // 目标装甲中心在世界坐标系中的位置
    Eigen::Quaterniond target_orientation; // 目标装甲方向(世界坐标系)

    // 计算字段(可选，方便使用)
    Eigen::Matrix3d target_rotation_matrix; // 提取的旋转矩阵
    bool is_valid = true;                   // 数据有效性标志
};

/**
 * @brief 包含连续角度处理的角度观测数据
 */
struct AngleObservation {
    double timestamp;        // Unix时间戳(秒)
    double absolute_angle;   // 从旋转矩阵得到的原始角度(弧度)
    double continuous_angle; // 跳变补偿后的连续角度(弧度)
    double velocity;         // 滤波后的速度 (rad/s)
    int blade_offset;        // 叶片切换偏移(72°的倍数)
};

/**
 * @brief 角度运动模型参数
 * 支持线性模型(小能量机关)和正弦波模型(大能量机关)
 */
struct AngleModel {
    bool is_big_rune; // true: 正弦波, false: 线性

    // 大能量机关正弦波模型: angle(t) = -a/ω·cos(ωt + t₀) + b·t + c
    // 其中 spd(t) = a·sin(ωt) + b, b = 2.090 - a
    double sin_amplitude;  // a: 振幅 [0.780, 1.045]
    double sin_omega;      // ω: 角频率 [1.884, 2.000] (rad/s)
    double sin_phase;      // t₀: 相位/时间偏移 (秒)
    double sin_offset;     // b: 速度偏移 (现在独立!)
    double sin_const_term; // c: 常数项 (弧度)

    // 线性模型: θ(t) = ω·t + b
    double lin_omega;  // ω: 角速度 (rad/s)
    double lin_offset; // b: 初始角度偏移 (弧度)

    // 元数据
    double fit_error;      // 拟合误差指标 (RMS)
    double last_fit_time;  // 最后一次参数更新的时间戳
    int data_points_used;  // 用于拟合的点数
    bool is_valid = false; // 模型有效性标志 (有足够数据时为true)

    // 运动状态跟踪
    double current_phase;         // 从当前速度实时计算的相位 φ = ωt + t₀
    double last_continuous_angle; // 最近观测的连续角度（积分起点）
    double last_observation_time; // 最近观测的时间戳
    double raw_velocity;          // 最近一次的原始速度观测 (调试用)

    // 外部滤波器提供的速度（来自 ISRCKF tracker）
    double filtered_velocity;   // 滤波后的角速度 (rad/s)
    bool velocity_filter_valid; // 外部滤波器是否有效

    // 默认构造函数，初始化为安全的默认值
    AngleModel()
        : is_big_rune(false)
        , sin_amplitude(0.9125)
        , // (0.780 + 1.045) / 2
        sin_omega(1.942)
        , // (1.884 + 2.000) / 2
        sin_phase(0.0)
        , sin_offset(1.1775)
        , // 2.090 - 0.9125
        sin_const_term(0.0)
        , lin_omega(0.0)
        , lin_offset(0.0)
        , fit_error(0.0)
        , last_fit_time(0.0)
        , data_points_used(0)
        , is_valid(false)
        , current_phase(0.0)
        , last_continuous_angle(0.0)
        , last_observation_time(0.0)
        , raw_velocity(0.0)
        , filtered_velocity(0.0)
        , velocity_filter_valid(false) {}
};

/**
 * @brief 预测结果
 */
struct PredictionResult {
    double predicted_angle;             // 预测的连续角度 (弧度)
    Eigen::Vector3d predicted_position; // 预测的目标位置(世界坐标系)
    bool is_valid;                      // 结果有效性标志

    // 调试信息
    double prediction_time_offset; // 用于预测的时间偏移 (秒)

    // 默认构造函数，初始化为安全的默认值
    PredictionResult()
        : predicted_angle(0.0)
        , predicted_position(Eigen::Vector3d::Zero())
        , is_valid(false)
        , prediction_time_offset(0.0) {}
};

/**
 * @brief 求解器配置参数
 */
struct EnergyMeterConfig {
    // 基本参数
    std::string rune_type;    // "big" 或 "small"
    double predict_time;      // 预测时间偏移 (秒)
    std::string gimbal_frame; // 目标云台参考坐标系
    std::string world_frame;  // 世界参考坐标系 (通常是 "odom")

    // 拟合参数
    double smooth_alpha;  // 参数平滑系数 (0-1)
    int min_data_points;  // 可靠拟合所需的最少点数

    double control_delay; // 控制系统延迟 (秒)
};

} // namespace energy_meter

#endif // ENERGY_METER_SOLVER_TYPES_HPP_
