#ifndef RM_GIMBAL_TRAJECTORY_PLANNER_HPP_
#define RM_GIMBAL_TRAJECTORY_PLANNER_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "rm_gimbal/trajectory_generator.hpp"
#include "types.hpp"  // TinyMPC types (includes TinySolver definition)

namespace rm_gimbal {

// MPC规划器参数
struct PlannerParams {
    int half_horizon{50};           // 半预测步数（总步数 = 2 * half_horizon）
    double dt{0.01};                // 时间步长（秒）

    // 云台动力学约束
    double max_acc_yaw{100.0};      // yaw最大加速度（rad/s²）
    double max_acc_pitch{100.0};    // pitch最大加速度（rad/s²）

    // 代价函数权重
    double Q_pos{100.0};            // 位置跟踪权重
    double Q_vel{1.0};              // 速度跟踪权重
    double R{0.1};                  // 控制量权重（加速度）

    // TinyMPC参数
    double rho{1.0};                // ADMM罚参数
    int max_iter{10};               // 最大迭代次数
    double abs_tol{1e-3};           // 绝对容差

    // 获取总预测步数
    int horizon() const { return half_horizon * 2; }
};

// MPC求解结果
struct PlannerSolution {
    std::vector<double> yaw_positions;
    std::vector<double> yaw_velocities;
    std::vector<double> yaw_accelerations;
    std::vector<double> pitch_positions;
    std::vector<double> pitch_velocities;
    std::vector<double> pitch_accelerations;

    // 参考轨迹（用于对比）
    std::vector<double> ref_yaw_positions;
    std::vector<double> ref_pitch_positions;

    bool solved{false};
    int iterations{0};
    double solve_time_ms{0.0};
    int half_horizon{0};  // 存储half_horizon用于getOutput

    // 获取当前时刻的输出（half_horizon位置）
    struct Output {
        double yaw{0.0};
        double yaw_vel{0.0};
        double yaw_acc{0.0};
        double pitch{0.0};
        double pitch_vel{0.0};
        double pitch_acc{0.0};
        // 参考值（用于判断跟踪误差和开火）
        double ref_yaw{0.0};
        double ref_pitch{0.0};
    };
    Output getOutput() const;
};

// 前馈缓存（用于BallisticSolver）
struct FeedforwardCache {
    double yaw_vel{0.0};
    double yaw_acc{0.0};
    double pitch_vel{0.0};
    double pitch_acc{0.0};
    bool valid{false};
};

class TrajectoryPlanner {
public:
    explicit TrajectoryPlanner(const PlannerParams& params);
    ~TrajectoryPlanner();

    // 初始化MPC求解器
    bool initialize();

    // 求解MPC
    // @param shooting_trajectory 射击轨迹（参考轨迹）
    // @param current_state_yaw [yaw, yaw_vel]
    // @param current_state_pitch [pitch, pitch_vel]
    // @return 求解结果
    PlannerSolution solve(
        const Trajectory& shooting_trajectory,
        const Eigen::Vector2d& current_state_yaw,
        const Eigen::Vector2d& current_state_pitch);

    // 更新约束参数（支持热更新）
    void updateConstraints(double max_acc_yaw, double max_acc_pitch);

    // 更新参数
    void updateParams(const PlannerParams& params);

    // 获取初始化状态
    bool isInitialized() const { return initialized_; }

private:
    // 设置单轴MPC
    bool setupMPC(TinySolver** solver, double max_acc) const;

    // 设置参考轨迹
    void setReference(TinySolver* solver, const Trajectory& traj, bool is_yaw) const;

    // 释放求解器
    void freeSolver(TinySolver** solver);

    // 提取解
    void extractSolution(TinySolver* solver, PlannerSolution& solution, bool is_yaw);

    PlannerParams params_;
    TinySolver* solver_yaw_{nullptr};
    TinySolver* solver_pitch_{nullptr};
    bool initialized_{false};
};

} // namespace rm_gimbal

#endif // RM_GIMBAL_TRAJECTORY_PLANNER_HPP_
