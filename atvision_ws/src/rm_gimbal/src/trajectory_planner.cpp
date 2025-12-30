#include "rm_gimbal/trajectory_planner.hpp"

#include <chrono>
#include <cmath>

// TinyMPC API
#include "tiny_api.hpp"

namespace rm_gimbal {

namespace {
// 将角度归一化到[-π, π]
inline double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}
} // namespace

// PlannerSolution methods
PlannerSolution::Output PlannerSolution::getOutput() const {
    Output out;
    // 注意：即使solved=false，也返回计算出的值（MPC可能未完全收敛但结果仍有意义）
    // 只有当数据为空时才返回默认值

    // 从half_horizon位置提取输出（当前时刻）
    const int idx = half_horizon;

    if (idx < static_cast<int>(yaw_positions.size())) {
        out.yaw = yaw_positions[idx];
        out.yaw_vel = yaw_velocities[idx];
    }
    if (idx < static_cast<int>(yaw_accelerations.size())) {
        out.yaw_acc = yaw_accelerations[idx];
    }

    if (idx < static_cast<int>(pitch_positions.size())) {
        out.pitch = pitch_positions[idx];
        out.pitch_vel = pitch_velocities[idx];
    }
    if (idx < static_cast<int>(pitch_accelerations.size())) {
        out.pitch_acc = pitch_accelerations[idx];
    }

    // 参考值
    if (idx < static_cast<int>(ref_yaw_positions.size())) {
        out.ref_yaw = ref_yaw_positions[idx];
    }
    if (idx < static_cast<int>(ref_pitch_positions.size())) {
        out.ref_pitch = ref_pitch_positions[idx];
    }

    return out;
}

// TrajectoryPlanner methods
TrajectoryPlanner::TrajectoryPlanner(const PlannerParams& params)
    : params_(params) {}

TrajectoryPlanner::~TrajectoryPlanner() {
    freeSolver(&solver_yaw_);
    freeSolver(&solver_pitch_);
}

void TrajectoryPlanner::freeSolver(TinySolver** solver) {
    if (*solver != nullptr) {
        delete (*solver)->work;
        delete (*solver)->settings;
        delete (*solver)->cache;
        delete (*solver)->solution;
        delete *solver;
        *solver = nullptr;
    }
}

bool TrajectoryPlanner::initialize() {
    // 释放旧的求解器
    freeSolver(&solver_yaw_);
    freeSolver(&solver_pitch_);

    // 初始化yaw轴MPC
    if (!setupMPC(&solver_yaw_, params_.max_acc_yaw)) {
        return false;
    }

    // 初始化pitch轴MPC
    if (!setupMPC(&solver_pitch_, params_.max_acc_pitch)) {
        freeSolver(&solver_yaw_);
        return false;
    }

    initialized_ = true;
    return true;
}

bool TrajectoryPlanner::setupMPC(TinySolver** solver, double max_acc) const {
    const int nx    = 2; // 状态维度 [angle, angular_velocity]
    const int nu    = 1; // 控制维度 [angular_acceleration]
    const int N     = params_.horizon();  // 使用总步数
    const double dt = params_.dt;

    // 1. 构建离散化动力学矩阵（简化模型，与Example一致）
    // x(k+1) = A * x(k) + B * u(k)
    // A = [1  dt]     B = [0 ]
    //     [0   1]         [dt]
    tinyMatrix Adyn(nx, nx);
    Adyn << 1.0, dt, 0.0, 1.0;

    tinyMatrix Bdyn(nx, nu);
    Bdyn << 0.0, dt;  // 简化模型：控制量直接影响速度

    tinyVector fdyn = tinyVector::Zero(nx);

    // 2. 构建代价矩阵
    // Q: 状态权重矩阵 (nx x nx)
    tinyMatrix Q(nx, nx);
    Q << params_.Q_pos, 0.0, 0.0, params_.Q_vel;

    // R: 控制权重矩阵 (nu x nu)
    tinyMatrix R(nu, nu);
    R << params_.R;

    // 3. 调用TinyMPC设置
    int status = tiny_setup(solver, Adyn, Bdyn, fdyn, Q, R, params_.rho, nx, nu, N, 0);
    if (status != 0) {
        return false;
    }

    // 4. 设置输入约束（加速度约束）
    tinyMatrix u_min(nu, N - 1);
    tinyMatrix u_max(nu, N - 1);
    u_min.setConstant(-max_acc);
    u_max.setConstant(max_acc);

    // 状态无界约束
    tinyMatrix x_min(nx, N);
    tinyMatrix x_max(nx, N);
    x_min.setConstant(-1e17);
    x_max.setConstant(1e17);

    tiny_set_bound_constraints(*solver, x_min, x_max, u_min, u_max);

    // 5. 设置求解器参数
    TinySettings* settings   = (*solver)->settings;
    settings->max_iter       = params_.max_iter;
    settings->abs_pri_tol    = params_.abs_tol;
    settings->abs_dua_tol    = params_.abs_tol;
    settings->en_input_bound = 1; // 启用输入约束
    settings->en_state_bound = 0; // 禁用状态约束

    return true;
}

void TrajectoryPlanner::setReference(TinySolver* solver, const Trajectory& traj, bool is_yaw) const {
    if (solver == nullptr)
        return;

    const int nx = 2;
    const int N  = params_.horizon();

    // 构建参考轨迹矩阵
    tinyMatrix x_ref(nx, N);
    for (int i = 0; i < N && i < static_cast<int>(traj.points.size()); ++i) {
        if (is_yaw) {
            x_ref(0, i) = traj.points[i].yaw;
            // 计算参考速度（中心差分，使用角度归一化避免±π边界问题）
            if (i > 0 && i < static_cast<int>(traj.points.size()) - 1) {
                double angle_diff = normalize_angle(traj.points[i + 1].yaw - traj.points[i - 1].yaw);
                x_ref(1, i) = angle_diff / (2 * traj.dt);
            } else if (i < static_cast<int>(traj.points.size()) - 1) {
                double angle_diff = normalize_angle(traj.points[i + 1].yaw - traj.points[i].yaw);
                x_ref(1, i) = angle_diff / traj.dt;
            } else if (i > 0) {
                x_ref(1, i) = x_ref(1, i - 1);
            } else {
                x_ref(1, i) = 0.0;
            }
        } else {
            x_ref(0, i) = traj.points[i].pitch;
            // pitch不需要角度归一化（范围通常在[-π/2, π/2]）
            if (i > 0 && i < static_cast<int>(traj.points.size()) - 1) {
                x_ref(1, i) = (traj.points[i + 1].pitch - traj.points[i - 1].pitch) / (2 * traj.dt);
            } else if (i < static_cast<int>(traj.points.size()) - 1) {
                x_ref(1, i) = (traj.points[i + 1].pitch - traj.points[i].pitch) / traj.dt;
            } else if (i > 0) {
                x_ref(1, i) = x_ref(1, i - 1);
            } else {
                x_ref(1, i) = 0.0;
            }
        }
    }

    // 如果轨迹点不够，填充最后一个值
    for (int i = static_cast<int>(traj.points.size()); i < N; ++i) {
        x_ref(0, i) = x_ref(0, traj.points.size() - 1);
        x_ref(1, i) = 0.0;
    }

    // 直接设置work->Xref（与Example一致）
    solver->work->Xref = x_ref;
}

void TrajectoryPlanner::extractSolution(
    TinySolver* solver, PlannerSolution& solution, bool is_yaw) {
    if (solver == nullptr || solver->work == nullptr)
        return;

    const int N = params_.horizon();

    // 提取状态序列（从work->x，而不是solution->x）
    const auto& x = solver->work->x; // nx x N
    const auto& u = solver->work->u; // nu x (N-1)

    if (is_yaw) {
        solution.yaw_positions.resize(N);
        solution.yaw_velocities.resize(N);
        solution.yaw_accelerations.resize(N - 1);

        for (int i = 0; i < N; ++i) {
            solution.yaw_positions[i]  = x(0, i);
            solution.yaw_velocities[i] = x(1, i);
        }
        for (int i = 0; i < N - 1; ++i) {
            solution.yaw_accelerations[i] = u(0, i);
        }

        // 存储参考轨迹用于对比
        solution.ref_yaw_positions.resize(N);
        for (int i = 0; i < N; ++i) {
            solution.ref_yaw_positions[i] = solver->work->Xref(0, i);
        }
    } else {
        solution.pitch_positions.resize(N);
        solution.pitch_velocities.resize(N);
        solution.pitch_accelerations.resize(N - 1);

        for (int i = 0; i < N; ++i) {
            solution.pitch_positions[i]  = x(0, i);
            solution.pitch_velocities[i] = x(1, i);
        }
        for (int i = 0; i < N - 1; ++i) {
            solution.pitch_accelerations[i] = u(0, i);
        }

        // 存储参考轨迹用于对比
        solution.ref_pitch_positions.resize(N);
        for (int i = 0; i < N; ++i) {
            solution.ref_pitch_positions[i] = solver->work->Xref(0, i);
        }
    }
}

PlannerSolution TrajectoryPlanner::solve(
    const Trajectory& shooting_trajectory, const Eigen::Vector2d& current_state_yaw,
    const Eigen::Vector2d& current_state_pitch) {
    // 关键改动：x0使用当前云台状态，而不是参考轨迹起点
    // 这样MPC会从当前状态规划到参考轨迹，产生平滑过渡

    PlannerSolution solution;
    solution.half_horizon = params_.half_horizon;

    if (!initialized_ || solver_yaw_ == nullptr || solver_pitch_ == nullptr) {
        solution.solved = false;
        return solution;
    }

    if (shooting_trajectory.points.empty()) {
        solution.solved = false;
        return solution;
    }

    auto t_start = std::chrono::high_resolution_clock::now();

    // 1. 设置yaw轴参考轨迹
    setReference(solver_yaw_, shooting_trajectory, true);

    // 2. 设置yaw轴初始状态 = 当前云台状态（关键！这样才能产生规划效果）
    tinyVector x0_yaw(2);
    x0_yaw << current_state_yaw(0), current_state_yaw(1);
    tiny_set_x0(solver_yaw_, x0_yaw);

    // 2.5 初始化所有线性代价向量
    solver_yaw_->work->q = -(solver_yaw_->work->Xref.array().colwise() * solver_yaw_->work->Q.array());
    solver_yaw_->work->r = -(solver_yaw_->work->Uref.array().colwise() * solver_yaw_->work->R.array());
    const int N_yaw = params_.horizon();
    solver_yaw_->work->p.col(N_yaw - 1) = -(solver_yaw_->work->Xref.col(N_yaw - 1).transpose() * solver_yaw_->cache->Pinf).transpose();

    // 3. 求解yaw轴MPC
    int status_yaw = tiny_solve(solver_yaw_);

    // 4. 提取yaw轴解
    extractSolution(solver_yaw_, solution, true);

    // 5. 设置pitch轴参考轨迹
    setReference(solver_pitch_, shooting_trajectory, false);

    // 6. 设置pitch轴初始状态 = 当前云台状态
    tinyVector x0_pitch(2);
    x0_pitch << current_state_pitch(0), current_state_pitch(1);
    tiny_set_x0(solver_pitch_, x0_pitch);

    // 6.5 初始化所有线性代价向量
    solver_pitch_->work->q = -(solver_pitch_->work->Xref.array().colwise() * solver_pitch_->work->Q.array());
    solver_pitch_->work->r = -(solver_pitch_->work->Uref.array().colwise() * solver_pitch_->work->R.array());
    const int N_pitch = params_.horizon();
    solver_pitch_->work->p.col(N_pitch - 1) = -(solver_pitch_->work->Xref.col(N_pitch - 1).transpose() * solver_pitch_->cache->Pinf).transpose();

    // 7. 求解pitch轴MPC
    int status_pitch = tiny_solve(solver_pitch_);

    // 8. 提取pitch轴解
    extractSolution(solver_pitch_, solution, false);

    // 9. 记录求解状态
    solution.solved     = (status_yaw == 0 && status_pitch == 0);
    solution.iterations = solver_yaw_->work->iter + solver_pitch_->work->iter;

    auto t_end             = std::chrono::high_resolution_clock::now();
    solution.solve_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    return solution;
}

void TrajectoryPlanner::updateConstraints(double max_acc_yaw, double max_acc_pitch) {
    params_.max_acc_yaw   = max_acc_yaw;
    params_.max_acc_pitch = max_acc_pitch;

    // 重新设置约束
    if (solver_yaw_ != nullptr) {
        const int nu = 1;
        const int N  = params_.horizon();

        tinyMatrix u_min(nu, N - 1);
        tinyMatrix u_max(nu, N - 1);
        u_min.setConstant(-max_acc_yaw);
        u_max.setConstant(max_acc_yaw);

        tinyMatrix x_min(2, N);
        tinyMatrix x_max(2, N);
        x_min.setConstant(-1e17);
        x_max.setConstant(1e17);

        tiny_set_bound_constraints(solver_yaw_, x_min, x_max, u_min, u_max);
    }

    if (solver_pitch_ != nullptr) {
        const int nu = 1;
        const int N  = params_.horizon();

        tinyMatrix u_min(nu, N - 1);
        tinyMatrix u_max(nu, N - 1);
        u_min.setConstant(-max_acc_pitch);
        u_max.setConstant(max_acc_pitch);

        tinyMatrix x_min(2, N);
        tinyMatrix x_max(2, N);
        x_min.setConstant(-1e17);
        x_max.setConstant(1e17);

        tiny_set_bound_constraints(solver_pitch_, x_min, x_max, u_min, u_max);
    }
}

void TrajectoryPlanner::updateParams(const PlannerParams& params) {
    bool need_reinit =
        (params.half_horizon != params_.half_horizon || params.dt != params_.dt
         || params.Q_pos != params_.Q_pos || params.Q_vel != params_.Q_vel || params.R != params_.R
         || params.rho != params_.rho);

    params_ = params;

    if (need_reinit && initialized_) {
        initialize();
    } else {
        updateConstraints(params.max_acc_yaw, params.max_acc_pitch);
    }
}

} // namespace rm_gimbal
