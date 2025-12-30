#ifndef RM_GIMBAL_SOLVER_PARAMS_HPP_
#define RM_GIMBAL_SOLVER_PARAMS_HPP_

#include <atomic>
#include <string>

namespace rm_gimbal {

struct TrajectoryParams {
    int iteration_times{20};
    double bullet_speed{25.0};
    double gravity{9.8};
    double resistance{0.001};
};

// MPC轨迹规划参数
struct MPCParams {
    bool enable{false};             // 是否启用MPC
    int half_horizon{50};           // 半预测步数（总步数 = 2 * half_horizon）
    double dt{0.01};                // 时间步长（秒）

    // 云台动力学约束（关键！必须设置为实际云台能力，否则MPC不会平滑）
    double max_acc_yaw{30.0};       // yaw最大加速度（rad/s²）- 实际云台约20-50
    double max_acc_pitch{30.0};     // pitch最大加速度（rad/s²）

    // 代价函数权重
    double Q_pos{100.0};            // 位置跟踪权重
    double Q_vel{1.0};              // 速度跟踪权重
    double R{1.0};                  // 控制量权重（增大可使轨迹更平滑）

    // TinyMPC参数
    double rho{1.0};                // ADMM罚参数
    int max_iter{10};               // 最大迭代次数
    double abs_tol{1e-3};           // 绝对容差

    // 轨迹生成参数
    double side_angle{15.0};        // 装甲板切换提前角度（度）
    double min_switching_v_yaw{1.0};// 触发切换判断的最小角速度（rad/s）

    // 获取总预测步数
    int horizon() const { return half_horizon * 2; }
};

struct SolverParams {
    double shooting_range_w{0.135};
    double shooting_range_h{0.135};
    double max_tracking_v_yaw{6.0};
    double prediction_delay{0.0};
    double side_angle{15.0};
    double min_switching_v_yaw{1.0};
    int transfer_thresh{5};

    TrajectoryParams trajectory;
    std::string compensator_type{"resistance"};

    // MPC参数
    MPCParams mpc;
};

// Thread-safe atomic parameters for hot-reload
struct AtomicSolverParams {
    std::atomic<double> max_tracking_v_yaw{6.0};
    std::atomic<double> prediction_delay{0.0};
    std::atomic<double> side_angle{15.0};
    std::atomic<double> min_switching_v_yaw{1.0};

    // MPC原子参数
    std::atomic<bool> mpc_enable{false};
    std::atomic<double> mpc_max_acc_yaw{100.0};
    std::atomic<double> mpc_max_acc_pitch{100.0};

    void update(const SolverParams& params) {
        max_tracking_v_yaw.store(params.max_tracking_v_yaw, std::memory_order_relaxed);
        prediction_delay.store(params.prediction_delay, std::memory_order_relaxed);
        side_angle.store(params.side_angle, std::memory_order_relaxed);
        min_switching_v_yaw.store(params.min_switching_v_yaw, std::memory_order_relaxed);

        // MPC参数
        mpc_enable.store(params.mpc.enable, std::memory_order_relaxed);
        mpc_max_acc_yaw.store(params.mpc.max_acc_yaw, std::memory_order_relaxed);
        mpc_max_acc_pitch.store(params.mpc.max_acc_pitch, std::memory_order_relaxed);
    }
};

} // namespace rm_gimbal

#endif // RM_GIMBAL_SOLVER_PARAMS_HPP_
