#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "energy_meter_solver/angle_fitter.hpp"

namespace energy_meter {

// ============================================================================
// 构造函数和析构函数
// ============================================================================

AngleFitter::AngleFitter(const EnergyMeterConfig& config)
    : config_(config)
    , min_data_points_(config.min_data_points)
    , max_observations_(100)           // 默认窗口大小，可从config提取
    , last_prediction_error_(0)
    , start_time_(0)
    , smooth_alpha_(config.smooth_alpha)
    , fit_interval_ms_(50)             // 拟合间隔默认50ms
    , last_angle_(0.0)                 // 初始化上一次角度
    , last_timestamp_(0.0)             // 初始化上一次时间戳
    , has_previous_observation_(false) // 初始化观测历史标志
    , last_real_observation_time_(0)   // 初始化最后真实观测时间
    , last_real_observation_angle_(0)  // 初始化最后真实观测角度
{
    current_model_.is_valid    = false;
    current_model_.is_big_rune = config.rune_type == "big";
}

AngleFitter::~AngleFitter() = default;

void AngleFitter::reset() {
    // 清空观测历史
    {
        std::unique_lock<std::shared_mutex> lock(observations_mutex_);
        observations_.clear();
        frame_interval_history_.clear();
    }

    // 重置模型状态
    {
        std::unique_lock<std::shared_mutex> lock(model_mutex_);
        current_model_             = AngleModel();
        current_model_.is_valid    = false;
        current_model_.is_big_rune = config_.rune_type == "big";
    }

    // 重置其他状态
    start_time_                  = 0.0;
    last_prediction_error_       = 0.0;
    last_angle_                  = 0.0;
    last_timestamp_              = 0.0;
    has_previous_observation_    = false;
    last_real_observation_time_  = 0.0;
    last_real_observation_angle_ = 0.0;
}

// ============================================================================
// addObservation - 仅负责数据收集，触发拟合
// ============================================================================

bool AngleFitter::addObservation(const AngleObservation& observation) {
    // 使用unique_lock保护observations_
    std::unique_lock<std::shared_mutex> lock(observations_mutex_);

    // 首次观测时初始化起始时间
    if (observations_.empty()) {
        start_time_ = observation.timestamp;
    } else {
        // 检查时间戳是否单调递增
        if (observation.timestamp < observations_.back().timestamp) {
            // 时间回退 - 直接抛弃这一帧，保持现有的模型和观测历史
            return false;
        }

        // 【新增】记录帧间隔用于计算平均帧率
        double frame_interval = observation.timestamp - observations_.back().timestamp;

        // 【改进】检测短暂丢失后恢复，用虚拟数据填补空白
        // 条件：间隔 > 100ms（不是正常帧）且 < 2s（不是长时间丢失）
        if (frame_interval > 0.1 && frame_interval < 2.0) {
            // 检查是否有有效模型可以生成虚拟数据
            bool has_valid_model = false;
            {
                lock.unlock(); // 临时释放observations_锁
                std::shared_lock<std::shared_mutex> model_lock(model_mutex_);
                has_valid_model = current_model_.is_valid;
            }
            lock.lock();       // 重新获取observations_锁

            if (has_valid_model && last_real_observation_time_ > 0) {
                double avg_interval = getAverageFrameInterval();
                if (avg_interval > 1e-6) {
                    // 释放锁并生成虚拟数据
                    lock.unlock();
                    generateSyntheticObservations(
                        last_real_observation_time_, observation.timestamp, avg_interval);
                    lock.lock();
                }
            }
        }

        // 正常帧间隔，记录用于计算平均帧率
        if (frame_interval > 0 && frame_interval < 0.1) {
            frame_interval_history_.push_back(frame_interval);
            if (frame_interval_history_.size() > MAX_FRAME_INTERVALS) {
                frame_interval_history_.pop_front();
            }
        }
    }

    // 【新增】保存当前观测为"最后的真实观测"
    last_real_observation_time_  = observation.timestamp;
    last_real_observation_angle_ = observation.continuous_angle;

    // 添加到历史数据(保留最近N个观测)
    observations_.push_back(observation);
    if (observations_.size() > max_observations_) {
        observations_.pop_front();
    }

    // 释放锁
    lock.unlock();

    // 更新模型中的最后观测信息
    {
        std::unique_lock<std::shared_mutex> model_lock(model_mutex_);
        current_model_.last_continuous_angle = observation.continuous_angle;
        current_model_.last_observation_time = observation.timestamp;
        current_model_.data_points_used      = static_cast<int>(observations_.size());
    }

    return false;
}

// ============================================================================
// 【新】最小二乘先验拟合角度参数
// ============================================================================

bool AngleFitter::fitAngleModelLSQ() {
    /*
     * 用滤波速度拟合参数
     *
     * 速度模型: v(t) = a·sin(ωt + φ) + b
     *
     * 三角展开:
     * a·sin(ωt + φ) = a·[sin(φ)·cos(ωt) + cos(φ)·sin(ωt)]
     *               = A·cos(ωt) + B·sin(ωt)
     * 其中:
     *   A = a·sin(φ)
     *   B = a·cos(φ)
     *
     * 线性模型 (固定ω):
     * v(t) = A·cos(ωt) + B·sin(ωt) + b
     *
     * 矩阵形式: V = H·X, 其中 X = [A, B, b]^T
     *
     * 反解:
     *   a = sqrt(A² + B²)
     *   φ = atan2(A, B)
     */

    std::shared_lock<std::shared_mutex> lock(observations_mutex_);

    if (observations_.size() < static_cast<size_t>(min_data_points_)) {
        return false;
    }

    // 提取数据：时间和滤波速度
    std::vector<double> t_values, v_values;
    for (const auto& obs : observations_) {
        t_values.push_back(getRelativeTimeNoLock(obs.timestamp));
        v_values.push_back(obs.velocity); // 使用滤波后的速度
    }

    lock.unlock();

    int n = static_cast<int>(t_values.size());
    if (n < 3)
        return false;                     // 至少需要3个点求解3个参数

    double best_error = std::numeric_limits<double>::max();
    double best_a     = AMPLITUDE_INIT;
    double best_omega = OMEGA_INIT;
    double best_phi   = 0.0;
    double best_b     = OFFSET_BASE - best_a;

    // 搜索 ω ∈ [1.884, 2.000]，步长0.005
    for (double omega = OMEGA_MIN; omega <= OMEGA_MAX + 1e-6; omega += 0.005) {

        // 构建设计矩阵 H (n×3)
        Eigen::MatrixXd H(n, 3);
        Eigen::VectorXd V(n);

        for (int i = 0; i < n; i++) {
            double t = t_values[i];
            H(i, 0)  = std::cos(omega * t); // A 的系数
            H(i, 1)  = std::sin(omega * t); // B 的系数
            H(i, 2)  = 1.0;                 // b 的系数
            V(i)     = v_values[i];
        }

        // 线性最小二乘解: X = (H'H)⁻¹ H'V
        Eigen::Matrix3d HtH = H.transpose() * H;
        Eigen::Vector3d HtV = H.transpose() * V;

        // 使用 LDLT 分解求解（数值稳定）
        Eigen::Vector3d X = HtH.ldlt().solve(HtV);

        double A = X(0);
        double B = X(1);
        double b = X(2);

        // 反解 a, φ
        // a = sqrt(A² + B²)
        // φ = atan2(A, B)
        double a   = std::sqrt(A * A + B * B);
        double phi = std::atan2(A, B);

        // 计算拟合误差
        Eigen::VectorXd residual = V - H * X;
        double error             = residual.squaredNorm() / n; // MSE

        if (error < best_error) {
            best_error = error;
            best_a     = a;
            best_omega = omega;
            best_phi   = phi;
            best_b     = b;
        }
    }

    // 根据最后一个观测点的时间和相位，反推 t₀
    // φ = ω·t + t₀  =>  t₀ = φ - ω·t_last
    double t_last  = t_values.back();
    double best_t0 = best_phi - best_omega * t_last;
    // 归一化 t₀ 到 [-π, π]
    best_t0 = std::atan2(std::sin(best_t0), std::cos(best_t0));

    // 计算常数项 c：从最后一个观测点的角度反推
    // θ(t) = -a/ω·cos(ωt + t₀) + b·t + c
    // c = θ_last - (-a/ω·cos(ω·t_last + t₀) + b·t_last)
    lock.lock();
    double theta_last = observations_.back().continuous_angle;
    lock.unlock();

    double best_c =
        theta_last
        - (-best_a / best_omega * std::cos(best_omega * t_last + best_t0) + best_b * t_last);

    // 保存先验参数到模型
    {
        std::unique_lock<std::shared_mutex> model_lock(model_mutex_);
        current_model_.sin_amplitude    = best_a;
        current_model_.sin_omega        = best_omega;
        current_model_.sin_phase        = best_t0;
        current_model_.sin_offset       = best_b;
        current_model_.sin_const_term   = best_c;
        current_model_.fit_error        = std::sqrt(best_error); // RMSE
        current_model_.is_big_rune      = true;
        current_model_.is_valid         = true;
        current_model_.data_points_used = n;
        current_model_.last_fit_time    = t_last + start_time_;
    }

    return true;

    // /*
    //  * 角度模型: θ(t) = -a/ω·cos(ωt + t₀) + b·t + c
    //  *
    //  * 三角展开:
    //  * -a/ω·cos(ωt + t₀) = -a/ω·[cos(t₀)·cos(ωt) - sin(t₀)·sin(ωt)]
    //  *                    = A·cos(ωt) + B·sin(ωt)
    //  * 其中:
    //  *   A = -a/ω·cos(t₀)
    //  *   B = a/ω·sin(t₀)
    //  *
    //  * 线性模型 (固定ω):
    //  * θ(t) = A·cos(ωt) + B·sin(ωt) + b·t + c
    //  *
    //  * 矩阵形式: Θ = H·X, 其中 X = [A, B, b, c]^T
    //  */

    // std::shared_lock<std::shared_mutex> lock(observations_mutex_);

    // if (observations_.size() < static_cast<size_t>(min_data_points_)) {
    //     return false;
    // }

    // // 提取数据
    // std::vector<double> t_values, theta_values;
    // for (const auto& obs : observations_) {
    //     t_values.push_back(getRelativeTimeNoLock(obs.timestamp));
    //     theta_values.push_back(obs.continuous_angle);
    // }

    // lock.unlock();

    // int n = static_cast<int>(t_values.size());
    // if (n < 4)
    //     return false; // 至少需要4个点求解4个参数

    // double best_error = std::numeric_limits<double>::max();
    // double best_a     = AMPLITUDE_INIT;
    // double best_omega = OMEGA_INIT;
    // double best_t0    = 0.0;
    // double best_b     = OFFSET_BASE - best_a;
    // double best_c     = 0.0;

    // // 搜索 ω ∈ [1.884, 2.000]，步长0.005
    // for (double omega = OMEGA_MIN; omega <= OMEGA_MAX + 1e-6; omega += 0.005) {

    //     // 构建设计矩阵 H (n×4)
    //     Eigen::MatrixXd H(n, 4);
    //     Eigen::VectorXd Theta(n);

    //     for (int i = 0; i < n; i++) {
    //         double t = t_values[i];
    //         H(i, 0)  = std::cos(omega * t); // A 的系数
    //         H(i, 1)  = std::sin(omega * t); // B 的系数
    //         H(i, 2)  = t;                   // b 的系数
    //         H(i, 3)  = 1.0;                 // c 的系数
    //         Theta(i) = theta_values[i];
    //     }

    //     // 线性最小二乘解: X = (H'H)⁻¹ H'Θ
    //     Eigen::Matrix4d HtH     = H.transpose() * H;
    //     Eigen::Vector4d HtTheta = H.transpose() * Theta;

    //     // 使用 LDLT 分解求解（数值稳定）
    //     Eigen::Vector4d X = HtH.ldlt().solve(HtTheta);

    //     double A = X(0);
    //     double B = X(1);
    //     double b = X(2);
    //     double c = X(3);

    //     // 反解 a, t₀
    //     // a/ω = sqrt(A² + B²)
    //     // t₀ = atan2(B, -A)
    //     double a_over_omega = std::sqrt(A * A + B * B);
    //     double a            = a_over_omega * omega;
    //     double t0           = std::atan2(B, -A);

    //     // 计算拟合误差
    //     Eigen::VectorXd residual = Theta - H * X;
    //     double error             = residual.squaredNorm() / n; // MSE

    //     if (error < best_error) {
    //         best_error = error;
    //         best_a     = a;
    //         best_omega = omega;
    //         best_t0    = t0;
    //         best_b     = b;
    //         best_c     = c;
    //     }
    // }

    // // 保存先验参数到模型
    // {
    //     std::unique_lock<std::shared_mutex> model_lock(model_mutex_);
    //     current_model_.sin_amplitude    = best_a;
    //     current_model_.sin_omega        = best_omega;
    //     current_model_.sin_phase        = best_t0;
    //     current_model_.sin_offset       = best_b;
    //     current_model_.sin_const_term   = best_c;
    //     current_model_.fit_error        = std::sqrt(best_error); // RMSE
    //     current_model_.is_big_rune      = true;
    //     current_model_.is_valid         = true;
    //     current_model_.data_points_used = n;
    //     current_model_.last_fit_time    = t_values.back() + start_time_;
    // }

    // return true;
}

// ============================================================================
// 【新】应用后验约束
// ============================================================================

void AngleFitter::applyPosteriorConstraints() {
    std::unique_lock<std::shared_mutex> lock(model_mutex_);

    if (!current_model_.is_valid || !current_model_.is_big_rune) {
        return;
    }

    // 约束1: a ∈ [0.780, 1.045]
    current_model_.sin_amplitude =
        std::clamp(current_model_.sin_amplitude, AMPLITUDE_MIN, AMPLITUDE_MAX);

    // 约束2: ω ∈ [1.884, 2.000]
    current_model_.sin_omega = std::clamp(current_model_.sin_omega, OMEGA_MIN, OMEGA_MAX);

    // 约束3: b = 2.090 - a (硬约束)
    current_model_.sin_offset = OFFSET_BASE - current_model_.sin_amplitude;
}

// ============================================================================
// 模型拟合 - 线性模型
// ============================================================================

bool AngleFitter::fitLinearModelPublic() {
    AngleModel linear_model;
    if (fitLinearModel(linear_model)) {
        linear_model.is_big_rune = false;

        std::unique_lock<std::shared_mutex> lock(model_mutex_);
        // 保留一些状态
        linear_model.last_continuous_angle = current_model_.last_continuous_angle;
        linear_model.last_observation_time = current_model_.last_observation_time;
        linear_model.filtered_velocity     = current_model_.filtered_velocity;
        linear_model.velocity_filter_valid = current_model_.velocity_filter_valid;
        linear_model.is_valid              = true;
        linear_model.data_points_used      = current_model_.data_points_used;

        current_model_ = linear_model;
        return true;
    }
    return false;
}

bool AngleFitter::fitLinearModel(AngleModel& model) {
    // 小能量机关使用最小二乘拟合: angle(t) = ω·t + b
    std::shared_lock<std::shared_mutex> lock(observations_mutex_);

    if (observations_.size() < static_cast<size_t>(min_data_points_)) {
        return false;
    }

    // 准备所有数据
    std::vector<double> all_t_values, all_angle_values;
    for (size_t i = 0; i < observations_.size(); i++) {
        all_t_values.push_back(getRelativeTimeNoLock(observations_[i].timestamp));
        all_angle_values.push_back(observations_[i].continuous_angle);
    }

    // 释放锁
    lock.unlock();

    // 对所有数据执行最小二乘回归
    double slope, intercept;
    if (!linearRegression(all_t_values, all_angle_values, slope, intercept)) {
        return false;
    }

    // 保存模型参数
    model.lin_omega   = slope;
    model.lin_offset  = intercept;
    model.is_big_rune = false; // 必须在计算误差前设置
    model.fit_error   = calculateFitError(model);

    return true;
}

bool AngleFitter::linearRegression(
    const std::vector<double>& x_values, const std::vector<double>& y_values, double& slope,
    double& intercept) {

    if (x_values.size() != y_values.size() || x_values.size() < 2) {
        return false;
    }

    int n        = x_values.size();
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;

    for (int i = 0; i < n; i++) {
        sum_x += x_values[i];
        sum_y += y_values[i];
        sum_xy += x_values[i] * y_values[i];
        sum_x2 += x_values[i] * x_values[i];
    }

    double denominator = n * sum_x2 - sum_x * sum_x;
    if (std::fabs(denominator) < 1e-10) {
        return false;
    }

    slope     = (n * sum_xy - sum_x * sum_y) / denominator;
    intercept = (sum_y - slope * sum_x) / n;

    return true;
}

// ============================================================================
// ============================================================================
// 短暂丧失恢复相关方法
// ============================================================================

double AngleFitter::getAverageFrameInterval() const {
    if (frame_interval_history_.empty()) {
        return 1.0 / 30.0; // 默认30Hz
    }

    double sum = 0.0;
    for (double interval : frame_interval_history_) {
        sum += interval;
    }
    return sum / frame_interval_history_.size();
}

void AngleFitter::generateSyntheticObservations(
    double loss_start_time, double loss_end_time, double avg_frame_interval) {
    std::unique_lock<std::shared_mutex> obs_lock(observations_mutex_);
    std::shared_lock<std::shared_mutex> model_lock(model_mutex_);

    if (!current_model_.is_valid || loss_end_time <= loss_start_time) {
        return;
    }

    // 计算需要填补的帧数
    double loss_duration     = loss_end_time - loss_start_time;
    int num_synthetic_frames = static_cast<int>(loss_duration / avg_frame_interval);

    // 限制虚拟帧数，避免过度填补
    if (num_synthetic_frames > 30) {
        num_synthetic_frames = 30;
    }

    if (num_synthetic_frames < 1) {
        return;
    }

    // 用拟合模型生成虚拟观测
    // 注意：需要使用窗口相对时间，与拟合时使用的时间基准一致
    double window_start_time =
        observations_.empty() ? start_time_ : observations_.front().timestamp;

    for (int i = 1; i <= num_synthetic_frames; i++) {
        double synthetic_time = loss_start_time + i * avg_frame_interval;

        // 用拟合模型预测此时刻的角度（使用窗口相对时间）
        double predicted_angle = 0;
        double t               = synthetic_time - window_start_time;
        if (current_model_.is_big_rune) {
            double a        = current_model_.sin_amplitude;
            double omega    = current_model_.sin_omega;
            double t0       = current_model_.sin_phase;
            double b        = current_model_.sin_offset;
            double c        = current_model_.sin_const_term;
            predicted_angle = -a / omega * std::cos(omega * t + t0) + b * t + c;
        } else {
            predicted_angle = current_model_.lin_omega * t + current_model_.lin_offset;
        }

        // 创建虚拟观测
        AngleObservation synthetic_obs;
        synthetic_obs.timestamp        = synthetic_time;
        synthetic_obs.continuous_angle = predicted_angle;
        // blade_offset 和 absolute_angle 保持不变（从最后一个真实观测继承）
        if (!observations_.empty()) {
            synthetic_obs.blade_offset   = observations_.back().blade_offset;
            synthetic_obs.absolute_angle = observations_.back().absolute_angle;
        } else {
            synthetic_obs.blade_offset   = 0;
            synthetic_obs.absolute_angle = 0;
        }

        observations_.push_back(synthetic_obs);

        // 维护观测窗口大小
        if (observations_.size() > max_observations_) {
            observations_.pop_front();
        }
    }
}

// ============================================================================
// 辅助方法
// ============================================================================

double AngleFitter::getRelativeTimeNoLock(double timestamp) const {
    if (!observations_.empty()) {
        return timestamp - observations_.front().timestamp;
    }
    return timestamp - start_time_;
}

double AngleFitter::calculateFitError(const AngleModel& model) {
    std::shared_lock<std::shared_mutex> lock(observations_mutex_);

    if (observations_.size() < 2) {
        return 0;
    }

    double sum_squared_error = 0;
    int count                = 0;

    // 使用角度增量计算误差
    for (size_t i = 1; i < observations_.size(); i++) {
        double dt = observations_[i].timestamp - observations_[i - 1].timestamp;
        double actual_delta =
            observations_[i].continuous_angle - observations_[i - 1].continuous_angle;
        double predicted_delta = 0;

        if (dt < 1e-6)
            continue; // 跳过时间间隔过小的点

        if (model.is_big_rune) {
            // 大能量机关：用速度积分计算角度增量
            // 从前一个观测的速度估算（用相邻点的角度差/时间差）
            double v_prev = 0;
            if (i >= 2) {
                double dt_prev = observations_[i - 1].timestamp - observations_[i - 2].timestamp;
                if (dt_prev > 1e-6) {
                    v_prev = (observations_[i - 1].continuous_angle
                              - observations_[i - 2].continuous_angle)
                           / dt_prev;
                }
            } else {
                v_prev = actual_delta / dt; // 第一个增量，用自身估算
            }

            double a     = model.sin_amplitude;
            double b     = model.sin_offset;
            double omega = model.sin_omega;

            // 从速度反推相位（和预测时一样的方式）
            double sin_phi = std::clamp((v_prev - b) / a, -1.0, 1.0);
            double phi_now = std::asin(sin_phi);

            // 角度增量 = -a/ω·cos(φ + ω·dt) + a/ω·cos(φ) + b·dt
            double phi_future = phi_now + omega * dt;
            predicted_delta =
                -a / omega * std::cos(phi_future) + a / omega * std::cos(phi_now) + b * dt;
        } else {
            // 小能量机关：线性模型，角度增量 = ω·dt
            predicted_delta = model.lin_omega * dt;
        }

        double error = actual_delta - predicted_delta;
        sum_squared_error += error * error;
        count++;
    }

    if (count == 0)
        return 0;
    return std::sqrt(sum_squared_error / count);

    // // 原始实现（基于绝对角度）
    // double sum_squared_error = 0;
    // int count                = 0;
    //
    // for (const auto& obs : observations_) {
    //     double t = getRelativeTimeNoLock(obs.timestamp);
    //     double predicted;
    //
    //     if (model.is_big_rune) {
    //         predicted = -model.sin_amplitude / model.sin_omega
    //                       * std::cos(model.sin_omega * t + model.sin_phase)
    //                   + model.sin_offset * t + model.sin_const_term;
    //     } else {
    //         predicted = model.lin_omega * t + model.lin_offset;
    //     }
    //
    //     double error = obs.continuous_angle - predicted;
    //     sum_squared_error += error * error;
    //     count++;
    // }
    //
    // if (count == 0)
    //     return 0;
    // return std::sqrt(sum_squared_error / count);
}

} // namespace energy_meter
