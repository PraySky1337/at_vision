#ifndef RM_GIMBAL_TRAJECTORY_GENERATOR_HPP_
#define RM_GIMBAL_TRAJECTORY_GENERATOR_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"

namespace rm_gimbal {

// 轨迹点结构
struct TrajectoryPoint {
    double time;           // 相对时间（秒）
    double yaw;            // yaw角（rad）
    double pitch;          // pitch角（rad）
    int armor_id;          // 当前装甲板ID
    bool is_switching;     // 是否为切换点（装甲板切换）
};

// 轨迹数据
struct Trajectory {
    std::vector<TrajectoryPoint> points;
    double dt;             // 时间步长（秒）
    int horizon;           // 预测步数

    // 获取参考轨迹矩阵用于MPC
    Eigen::MatrixXd getYawRefMatrix() const;
    Eigen::MatrixXd getPitchRefMatrix() const;
};

// 轨迹生成器参数
struct GeneratorParams {
    int half_horizon{50};               // 半预测步数（总步数 = 2 * half_horizon）
    double dt{0.01};                    // 时间步长（秒）
    double side_angle{15.0};            // 装甲板切换提前角度（度）
    double min_switching_v_yaw{1.0};    // 触发切换判断的最小角速度（rad/s）

    // 获取总预测步数
    int horizon() const { return half_horizon * 2; }
};

class TrajectoryGenerator {
public:
    explicit TrajectoryGenerator(const GeneratorParams& params);
    ~TrajectoryGenerator() = default;

    // 生成射击轨迹（目标轨迹提前t_fly）
    // @param target 目标消息
    // @param muzzle_position 枪口位置（世界坐标系）
    // @param bullet_speed 子弹速度（m/s）
    // @param trajectory_compensator 弹道补偿器（用于计算飞行时间）
    // @return 射击轨迹
    Trajectory generateShootingTrajectory(
        const rm_interfaces::msg::Target& target,
        const Eigen::Vector3d& muzzle_position,
        double bullet_speed,
        const std::unique_ptr<fyt::TrajectoryCompensator>& trajectory_compensator);

    // 更新参数
    void updateParams(const GeneratorParams& params);

private:
    // 预测t秒后的目标状态
    struct PredictedState {
        Eigen::Vector3d position;  // 目标中心位置
        double yaw;                // 目标yaw角
    };
    PredictedState predictTargetState(
        const rm_interfaces::msg::Target& target,
        double dt) const;

    // 获取装甲板位置
    std::vector<Eigen::Vector4d> getArmorPositions(
        const rm_interfaces::msg::Target& target,
        const PredictedState& state) const;

    // 选择最佳装甲板
    // @return 装甲板索引 {0, 1, ..., armors_num-1}
    int selectBestArmor(
        const std::vector<Eigen::Vector4d>& armor_positions,
        const Eigen::Vector3d& muzzle_position,
        double target_yaw,
        double target_v_yaw,
        size_t armors_num) const;

    // 计算子弹飞行时间
    double calculateFlyingTime(
        const Eigen::Vector3d& armor_position,
        const Eigen::Vector3d& muzzle_position,
        double bullet_speed,
        const std::unique_ptr<fyt::TrajectoryCompensator>& trajectory_compensator) const;

    // 计算yaw和pitch角（无弹道补偿，仅用于参考轨迹）
    void calculateYawPitch(
        const Eigen::Vector3d& armor_position,
        const Eigen::Vector3d& muzzle_position,
        double& yaw,
        double& pitch) const;

    GeneratorParams params_;
};

} // namespace rm_gimbal

#endif // RM_GIMBAL_TRAJECTORY_GENERATOR_HPP_
