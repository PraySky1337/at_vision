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
    , auto_detect_type_(config.auto_detect_type)
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

    // 启动异步拟合线程
    stop_thread_   = false;
    valid_params_  = false;
    fit_triggered_ = false;

    fitting_thread_ = std::thread(&AngleFitter::fitThreadMain, this);
}

AngleFitter::~AngleFitter() {
    // 停止拟合线程
    stop_thread_ = true;

    // 唤醒线程以便退出
    {
        std::lock_guard<std::mutex> lock(fit_signal_mutex_);
        fit_triggered_ = true;
    }
    fit_signal_.notify_one();

    // 等待线程结束
    if (fitting_thread_.joinable()) {
        fitting_thread_.join();
    }
}

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
        current_model_ = AngleModel();
        current_model_.is_valid = false;
        current_model_.is_big_rune = config_.rune_type == "big";
    }

    // 重置其他状态
    start_time_ = 0.0;
    last_prediction_error_ = 0.0;
    last_angle_ = 0.0;
    last_timestamp_ = 0.0;
    has_previous_observation_ = false;
    last_real_observation_time_ = 0.0;
    last_real_observation_angle_ = 0.0;
    valid_params_ = false;
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
                lock.unlock();  // 临时释放observations_锁
                std::shared_lock<std::shared_mutex> model_lock(model_mutex_);
                has_valid_model = current_model_.is_valid;
            }
            lock.lock();  // 重新获取observations_锁

            if (has_valid_model && last_real_observation_time_ > 0) {
                double avg_interval = getAverageFrameInterval();
                if (avg_interval > 1e-6) {
                    // 释放锁并生成虚拟数据
                    lock.unlock();
                    generateSyntheticObservations(
                        last_real_observation_time_,
                        observation.timestamp,
                        avg_interval);
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

    // 只在数据足够时触发拟合
    if (observations_.size() >= static_cast<size_t>(min_data_points_)) {
        triggerFitting();
        return true;
    }

    return false;
}

// ============================================================================
// 异步拟合线程
// ============================================================================

void AngleFitter::triggerFitting() {
    // 设置拟合触发标志并唤醒拟合线程
    {
        std::lock_guard<std::mutex> lock(fit_signal_mutex_);
        fit_triggered_ = true;
    }
    fit_signal_.notify_one();
}

void AngleFitter::fitThreadMain() {
    // 类似Calculate.cpp的fit()函数，后台线程主循环
    while (!stop_thread_) {
        // 等待拟合信号
        {
            std::unique_lock<std::mutex> lock(fit_signal_mutex_);
            fit_signal_.wait(lock, [this] { return fit_triggered_.load() || stop_thread_.load(); });

            if (stop_thread_) {
                break;
            }

            // 重置触发标志
            fit_triggered_ = false;
        }

        // 执行拟合
        bool success = performFitting();

        // 更新valid_params_标志
        if (success) {
            valid_params_ = true;
        }

        // 添加小延迟避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(fit_interval_ms_));
    }
}

bool AngleFitter::performFitting() {
    // 复制observations_到本地副本（使用shared_lock）
    std::deque<AngleObservation> local_observations;

    {
        std::shared_lock<std::shared_mutex> lock(observations_mutex_);
        local_observations = observations_;
    }

    // 检查数据是否足够
    if (local_observations.size() < static_cast<size_t>(min_data_points_)) {
        return false;
    }

    // 完整的角度拟合
    AngleModel new_model;
    bool fit_success = false;

    // 默认尝试大符模型
    fit_success = fitSinusoidalModel(new_model);
    new_model.is_big_rune = true;

    // 基于误差的自动检测：如果大符误差太大，切换到小符
    constexpr double ERROR_THRESHOLD = 0.5;  // 误差阈值 (rad)
    if (auto_detect_type_ && fit_success && new_model.fit_error > ERROR_THRESHOLD) {
        // 大符误差太大，尝试小符
        AngleModel linear_model;
        if (fitLinearModel(linear_model)) {
            // 比较两个模型的误差，选择更好的
            if (linear_model.fit_error < new_model.fit_error) {
                new_model = linear_model;
                new_model.is_big_rune = false;
            }
        }
    }

    if (!fit_success) {
        // 大符拟合失败，尝试小符
        fit_success = fitLinearModel(new_model);
        new_model.is_big_rune = false;
    }

    if (!fit_success) {
        return false;
    }

    // 使用unique_lock更新current_model_
    {
        std::unique_lock<std::shared_mutex> lock(model_mutex_);

        double saved_last_continuous_angle = current_model_.last_continuous_angle;
        double saved_last_observation_time = current_model_.last_observation_time;
        double saved_current_phase         = current_model_.current_phase;
        double saved_raw_velocity          = current_model_.raw_velocity;

        // 平滑参数
        if (current_model_.is_valid) {
            smoothModelParameters(new_model);
        } else {
            // 首次拟合，直接使用新模型
            current_model_ = new_model;
        }

        current_model_.last_continuous_angle = saved_last_continuous_angle;
        current_model_.last_observation_time = saved_last_observation_time;
        current_model_.current_phase         = saved_current_phase;
        current_model_.raw_velocity          = saved_raw_velocity;

        // 更新模型状态
        current_model_.is_valid         = true;
        current_model_.last_fit_time    = local_observations.back().timestamp;
        current_model_.data_points_used = local_observations.size();
    }

    // 计算并更新last_prediction_error_
    last_prediction_error_ = new_model.fit_error;

    return true;
}

// ============================================================================
// 模型拟合 - 线性模型
// ============================================================================

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
    model.lin_omega  = slope;
    model.lin_offset = intercept;
    model.is_big_rune = false;  // 必须在计算误差前设置
    model.fit_error  = calculateFitError(model);

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
// 模型拟合 - 正弦波模型（改进版）
// ============================================================================

bool AngleFitter::fitSinusoidalModel(AngleModel& model) {
    // 【简化】直接使用网格搜索 + LM优化
    // 旧方法使用 initializeParametersFromVelocity() 已删除

    // 准备数据
    std::vector<int> indices;
    {
        std::shared_lock<std::shared_mutex> lock(observations_mutex_);
        size_t obs_size = observations_.size();

        if (obs_size > 50) {
            indices = adaptiveRansacOutlierRejection();
        } else {
            indices = rejectOutliers();
        }

        if (indices.size() < static_cast<size_t>(min_data_points_)) {
            // 数据不足
            return false;
        }
    }

    // 提取观测数据
    std::vector<double> t_values, angle_values;
    {
        std::shared_lock<std::shared_mutex> lock(observations_mutex_);
        for (int idx : indices) {
            t_values.push_back(getRelativeTimeNoLock(observations_[idx].timestamp));
            angle_values.push_back(observations_[idx].continuous_angle);
        }
    }

    if (angle_values.empty()) {
        return false;
    }

    // 数据标准化
    double angle_offset = angle_values[0];
    for (auto& angle : angle_values) {
        angle -= angle_offset;
    }

    // 【改进】两阶段网格搜索初值
    double best_a = AMPLITUDE_INIT;
    double best_omega = OMEGA_INIT;
    double best_t0 = 0.0;
    double best_c = 0.0;

    if (!twoStageGridSearch(t_values, angle_values, best_a, best_omega, best_t0, best_c)) {
        return false;
    }

    // LM优化细化
    double a = best_a;
    double omega = best_omega;
    double t0 = best_t0;
    double c = best_c;
    double b = OFFSET_BASE - a; // 强制约束

    if (levenbergMarquardtOptimization(t_values, angle_values, a, omega, t0, c)) {
        // LM优化成功
        c += angle_offset; // 反标准化

        // 设置模型
        model.sin_amplitude  = a;
        model.sin_omega      = omega;
        model.sin_phase      = t0;
        model.sin_offset     = b;
        model.sin_const_term = c;
        model.is_big_rune    = true;

        enforceConstraints(model);
        model.fit_error = calculateFitError(model);

        return true;
    }

    // 回退：使用网格搜索结果
    model.sin_amplitude  = a;
    model.sin_omega      = omega;
    model.sin_phase      = t0;
    model.sin_offset     = b;
    model.sin_const_term = c;
    model.is_big_rune    = true;

    enforceConstraints(model);
    model.fit_error = calculateFitError(model);

    return true;
}

// ============================================================================
// 【新】两阶段网格搜索
// ============================================================================

bool AngleFitter::twoStageGridSearch(
    const std::vector<double>& t_values, const std::vector<double>& angle_values, double& best_a,
    double& best_omega, double& best_t0, double& best_c) {

    if (t_values.size() < static_cast<size_t>(min_data_points_)) {
        return false;
    }

    double best_error = std::numeric_limits<double>::max();

    // ===== 阶段1: 粗搜索 =====
    // 步长: a=0.1, ω=0.05, t0=0.5
    // 搜索空间: 3×3×13 ≈ 117 组合（比原来的数百组合少很多）

    double coarse_best_a     = AMPLITUDE_INIT;
    double coarse_best_omega = OMEGA_INIT;
    double coarse_best_t0    = 0.0;
    double coarse_best_c     = angle_values[0];

    for (double a = AMPLITUDE_MIN; a <= AMPLITUDE_MAX + 1e-6; a += GRID_COARSE_STEP_A) {
        for (double omega = OMEGA_MIN; omega <= OMEGA_MAX + 1e-6; omega += GRID_COARSE_STEP_OMEGA) {
            double b = OFFSET_BASE - a; // 强制约束

            // 对于每组(a, ω)，尝试多个t₀值
            for (double t0 = -M_PI; t0 <= M_PI + 1e-6; t0 += 0.5) {
                // 用最小二乘法求解c
                double sum_residual = 0.0;
                for (size_t i = 0; i < t_values.size(); i++) {
                    double t           = t_values[i];
                    double osc_part    = -a / omega * std::cos(omega * t + t0);
                    double linear_part = b * t;
                    sum_residual += angle_values[i] - osc_part - linear_part;
                }
                double c_opt = sum_residual / t_values.size();

                // 计算误差
                double error = 0.0;
                for (size_t i = 0; i < t_values.size(); i++) {
                    double t         = t_values[i];
                    double predicted = -a / omega * std::cos(omega * t + t0) + b * t + c_opt;
                    double residual  = angle_values[i] - predicted;
                    error += residual * residual;
                }
                error = std::sqrt(error / t_values.size());

                if (error < best_error) {
                    best_error        = error;
                    coarse_best_a     = a;
                    coarse_best_omega = omega;
                    coarse_best_t0    = t0;
                    coarse_best_c     = c_opt;
                }
            }
        }
    }

    // ===== 阶段2: 细搜索 =====
    // 只在粗搜索最优点附近搜索
    // 步长: a=0.02, ω=0.01, t0=0.2
    // 搜索范围: ±2步（3×3×5 = 45组合）

    double fine_a_min = std::max(AMPLITUDE_MIN, coarse_best_a - 2 * GRID_FINE_STEP_A);
    double fine_a_max = std::min(AMPLITUDE_MAX, coarse_best_a + 2 * GRID_FINE_STEP_A);

    double fine_omega_min = std::max(OMEGA_MIN, coarse_best_omega - 2 * GRID_FINE_STEP_OMEGA);
    double fine_omega_max = std::min(OMEGA_MAX, coarse_best_omega + 2 * GRID_FINE_STEP_OMEGA);

    double fine_t0_min = coarse_best_t0 - 0.4;
    double fine_t0_max = coarse_best_t0 + 0.4;

    for (double a = fine_a_min; a <= fine_a_max + 1e-6; a += GRID_FINE_STEP_A) {
        for (double omega = fine_omega_min; omega <= fine_omega_max + 1e-6;
             omega += GRID_FINE_STEP_OMEGA) {
            double b = OFFSET_BASE - a;

            for (double t0 = fine_t0_min; t0 <= fine_t0_max + 1e-6; t0 += 0.2) {
                // 用最小二乘法求解c
                double sum_residual = 0.0;
                for (size_t i = 0; i < t_values.size(); i++) {
                    double t           = t_values[i];
                    double osc_part    = -a / omega * std::cos(omega * t + t0);
                    double linear_part = b * t;
                    sum_residual += angle_values[i] - osc_part - linear_part;
                }
                double c_opt = sum_residual / t_values.size();

                // 计算误差
                double error = 0.0;
                for (size_t i = 0; i < t_values.size(); i++) {
                    double t         = t_values[i];
                    double predicted = -a / omega * std::cos(omega * t + t0) + b * t + c_opt;
                    double residual  = angle_values[i] - predicted;
                    error += residual * residual;
                }
                error = std::sqrt(error / t_values.size());

                if (error < best_error) {
                    best_error = error;
                    best_a     = a;
                    best_omega = omega;
                    best_t0    = t0;
                    best_c     = c_opt;
                }
            }
        }
    }

    // 如果细搜索没有改善，使用粗搜索结果
    if (best_error >= std::numeric_limits<double>::max() / 2) {
        best_a     = coarse_best_a;
        best_omega = coarse_best_omega;
        best_t0    = coarse_best_t0;
        best_c     = coarse_best_c;
    }

    return true;
}

// ============================================================================
// 【新】自适应RANSAC离群点剔除
// ============================================================================

std::vector<int> AngleFitter::adaptiveRansacOutlierRejection() {
    std::shared_lock<std::shared_mutex> lock(observations_mutex_);

    // 初始化所有索引
    std::vector<int> all_indices;
    for (size_t i = 0; i < observations_.size(); i++) {
        all_indices.push_back(i);
    }

    if (observations_.size() < static_cast<size_t>(min_data_points_)) {
        return all_indices;
    }

    std::vector<int> current_inliers = all_indices;
    const double outlier_threshold   = 0.5; // rad 内点阈值

    // 自适应迭代：初始3次，根据内点比例决定是否继续
    int current_iteration = 0;
    int max_iterations    = RANSAC_INITIAL_ITER;

    while (current_iteration < max_iterations && current_iteration < RANSAC_MAX_ITER) {
        // 步骤1：从当前内点中随机采样
        std::vector<int> sample_indices;
        int sample_count =
            std::min(static_cast<int>(min_data_points_), static_cast<int>(current_inliers.size()));

        // 使用mt19937随机选择采样点
        std::vector<int> shuffled_inliers = current_inliers;
        std::shuffle(shuffled_inliers.begin(), shuffled_inliers.end(), random_engine_);
        sample_indices.assign(shuffled_inliers.begin(), shuffled_inliers.begin() + sample_count);

        lock.unlock();

        // 步骤2：基于样本拟合并重新分类
        std::vector<int> new_inliers =
            ransacIterateOnce(sample_indices, all_indices, outlier_threshold);

        lock.lock();

        // 步骤3：计算内点比例
        double inlier_ratio = static_cast<double>(new_inliers.size()) / all_indices.size();

        // 步骤4：自适应决策
        if (inlier_ratio > RANSAC_INLIER_RATIO_THRESHOLD) {
            // 内点比例足够高，停止迭代
            current_inliers = new_inliers;
            break;
        }

        // 步骤5：如果有改善，更新内点集合
        if (new_inliers.size() > current_inliers.size()) {
            current_inliers = new_inliers;
        }

        current_iteration++;
    }

    return current_inliers;
}

std::vector<int> AngleFitter::ransacIterateOnce(
    const std::vector<int>& sample_indices, const std::vector<int>& all_indices,
    double outlier_threshold) {

    std::shared_lock<std::shared_mutex> lock(observations_mutex_);

    if (sample_indices.size() < static_cast<size_t>(min_data_points_)) {
        return sample_indices;
    }

    std::vector<double> sample_t, sample_angle;
    for (int idx : sample_indices) {
        sample_t.push_back(getRelativeTimeNoLock(observations_[idx].timestamp));
        sample_angle.push_back(observations_[idx].continuous_angle);
    }

    // 快速拟合临时模型（粗网格搜索）
    double best_error = std::numeric_limits<double>::max();
    AngleModel temp_model;

    for (double a = AMPLITUDE_MIN; a <= AMPLITUDE_MAX; a += 0.1) {
        for (double omega = OMEGA_MIN; omega <= OMEGA_MAX; omega += 0.1) {
            double b  = OFFSET_BASE - a;
            double t0 = 0.0;

            // 计算这组参数对样本的拟合误差
            double sample_error = 0.0;
            double sum_residual = 0.0;

            for (size_t i = 0; i < sample_t.size(); i++) {
                double osc = -a / omega * std::cos(omega * sample_t[i] + t0);
                double lin = b * sample_t[i];
                sum_residual += sample_angle[i] - osc - lin;
            }
            double c_opt = sum_residual / sample_t.size();

            for (size_t i = 0; i < sample_t.size(); i++) {
                double pred =
                    -a / omega * std::cos(omega * sample_t[i] + t0) + b * sample_t[i] + c_opt;
                double res = sample_angle[i] - pred;
                sample_error += res * res;
            }

            if (sample_error < best_error) {
                best_error                = sample_error;
                temp_model.sin_amplitude  = a;
                temp_model.sin_omega      = omega;
                temp_model.sin_offset     = b;
                temp_model.sin_phase      = t0;
                temp_model.sin_const_term = c_opt;
            }
        }
    }

    // 使用该模型对所有点进行分类
    std::vector<int> inliers;
    for (int idx : all_indices) {
        double t         = getRelativeTimeNoLock(observations_[idx].timestamp);
        double predicted = -temp_model.sin_amplitude / temp_model.sin_omega
                             * std::cos(temp_model.sin_omega * t + temp_model.sin_phase)
                         + temp_model.sin_offset * t + temp_model.sin_const_term;

        double error = std::fabs(observations_[idx].continuous_angle - predicted);
        if (error < outlier_threshold) {
            inliers.push_back(idx);
        }
    }

    // 如果内点太少，返回原样本
    if (inliers.size() < static_cast<size_t>(min_data_points_)) {
        return sample_indices;
    }

    return inliers;
}

// ============================================================================
// 【新】LM优化（改进版：减少迭代次数，集成约束）
// ============================================================================

bool AngleFitter::levenbergMarquardtOptimization(
    const std::vector<double>& t_values, const std::vector<double>& angle_values, double& a,
    double& omega, double& t0, double& c) {

    const int max_lm_iterations = 20; // 从50减少到20
    const double lm_lambda_init = 0.01;
    double lm_lambda            = lm_lambda_init;
    const double lm_lambda_up   = 10.0;
    const double lm_lambda_down = 0.1;

    // 4个参数: a, omega, t0, c (b由约束自动确定)
    std::vector<double> params = {a, omega, t0, c};
    int n_params               = params.size();
    int n_data                 = t_values.size();

    for (int iter = 0; iter < max_lm_iterations; iter++) {
        // 计算雅可比矩阵和残差
        std::vector<double> residuals(n_data);
        std::vector<std::vector<double>> jacobian(n_data, std::vector<double>(n_params, 0.0));

        a        = params[0];
        omega    = params[1];
        t0       = params[2];
        c        = params[3];
        double b = OFFSET_BASE - a; // 约束

        double current_error = 0.0;

        for (int i = 0; i < n_data; i++) {
            double t         = t_values[i];
            double predicted = -a / omega * std::cos(omega * t + t0) + b * t + c;
            residuals[i]     = angle_values[i] - predicted;
            current_error += residuals[i] * residuals[i];

            // 雅可比矩阵
            jacobian[i][0] = -(1.0 / omega * std::cos(omega * t + t0)) - t;
            jacobian[i][1] = a / (omega * omega) * std::cos(omega * t + t0)
                           - a / omega * t * std::sin(omega * t + t0);
            jacobian[i][2] = a / omega * std::sin(omega * t + t0);
            jacobian[i][3] = 1.0;
        }

        current_error = std::sqrt(current_error / n_data);

        // 构建正规方程: (J^T·J + λ·I)·δp = J^T·r
        std::vector<std::vector<double>> JtJ(n_params, std::vector<double>(n_params, 0.0));
        std::vector<double> Jtr(n_params, 0.0);

        for (int i = 0; i < n_data; i++) {
            for (int j = 0; j < n_params; j++) {
                Jtr[j] += jacobian[i][j] * residuals[i];
                for (int k = 0; k < n_params; k++) {
                    JtJ[j][k] += jacobian[i][j] * jacobian[i][k];
                }
            }
        }

        // 添加阻尼项和惩罚项（集成到目标函数中）
        for (int i = 0; i < n_params; i++) {
            JtJ[i][i] += lm_lambda;
        }

        // 高斯消元求解
        std::vector<double> delta_p(n_params);
        bool solve_ok = true;

        for (int col = 0; col < n_params; col++) {
            int pivot = col;
            for (int row = col + 1; row < n_params; row++) {
                if (std::fabs(JtJ[row][col]) > std::fabs(JtJ[pivot][col])) {
                    pivot = row;
                }
            }

            if (std::fabs(JtJ[pivot][col]) < 1e-10) {
                solve_ok = false;
                break;
            }

            std::swap(JtJ[col], JtJ[pivot]);
            std::swap(Jtr[col], Jtr[pivot]);

            for (int row = col + 1; row < n_params; row++) {
                double factor = JtJ[row][col] / JtJ[col][col];
                for (int k = col; k < n_params; k++) {
                    JtJ[row][k] -= factor * JtJ[col][k];
                }
                Jtr[row] -= factor * Jtr[col];
            }
        }

        if (!solve_ok) {
            break;
        }

        // 回代求解
        for (int i = n_params - 1; i >= 0; i--) {
            delta_p[i] = Jtr[i];
            for (int j = i + 1; j < n_params; j++) {
                delta_p[i] -= JtJ[i][j] * delta_p[j];
            }
            delta_p[i] /= JtJ[i][i];
        }

        // 更新参数并立即应用约束
        std::vector<double> new_params = params;
        new_params[0] += delta_p[0];
        new_params[1] += delta_p[1];
        new_params[2] += delta_p[2];
        new_params[3] += delta_p[3];

        // 强制应用约束
        if (new_params[0] < AMPLITUDE_MIN)
            new_params[0] = AMPLITUDE_MIN;
        if (new_params[0] > AMPLITUDE_MAX)
            new_params[0] = AMPLITUDE_MAX;
        if (new_params[1] < OMEGA_MIN)
            new_params[1] = OMEGA_MIN;
        if (new_params[1] > OMEGA_MAX)
            new_params[1] = OMEGA_MAX;

        // 计算新的误差
        double new_error = 0.0;
        for (int i = 0; i < n_data; i++) {
            double t         = t_values[i];
            double a_new     = new_params[0];
            double omega_new = new_params[1];
            double b_new     = OFFSET_BASE - a_new;
            double t0_new    = new_params[2];
            double c_new     = new_params[3];

            double predicted =
                -a_new / omega_new * std::cos(omega_new * t + t0_new) + b_new * t + c_new;
            double residual = angle_values[i] - predicted;
            new_error += residual * residual;
        }
        new_error = std::sqrt(new_error / n_data);

        // LM步骤决策
        if (new_error < current_error) {
            params = new_params;
            lm_lambda *= lm_lambda_down;
            if (std::fabs(new_error - current_error) < 1e-8) {
                break; // 收敛
            }
        } else {
            lm_lambda *= lm_lambda_up;
        }
    }

    // 输出优化后的参数
    a     = params[0];
    omega = params[1];
    t0    = params[2];
    c     = params[3];

    return true;
}

// ============================================================================
// 离群点剔除（简单版本，用于小数据集）
// ============================================================================

std::vector<int> AngleFitter::rejectOutliers() {
    std::shared_lock<std::shared_mutex> lock(observations_mutex_);

    std::vector<int> valid_indices;

    if (observations_.size() < static_cast<size_t>(min_data_points_)) {
        for (size_t i = 0; i < observations_.size(); i++) {
            valid_indices.push_back(i);
        }
        return valid_indices;
    }

    double max_allowed_jump = 0.5; // rad per frame
    valid_indices.push_back(0);

    for (size_t i = 1; i < observations_.size(); i++) {
        double angle_diff =
            std::fabs(observations_[i].continuous_angle - observations_[i - 1].continuous_angle);

        if (angle_diff < max_allowed_jump) {
            valid_indices.push_back(i);
        }
    }

    // 如果剔除太多，放宽阈值
    if (valid_indices.size() < static_cast<size_t>(min_data_points_)) {
        valid_indices.clear();
        max_allowed_jump = 1.0;

        valid_indices.push_back(0);
        for (size_t i = 1; i < observations_.size(); i++) {
            double angle_diff = std::fabs(
                observations_[i].continuous_angle - observations_[i - 1].continuous_angle);

            if (angle_diff < max_allowed_jump) {
                valid_indices.push_back(i);
            }
        }
    }

    // 如果还是不够，全部保留
    if (valid_indices.size() < static_cast<size_t>(min_data_points_)) {
        valid_indices.clear();
        for (size_t i = 0; i < observations_.size(); i++) {
            valid_indices.push_back(i);
        }
    }

    return valid_indices;
}

// ============================================================================
// 辅助函数
// ============================================================================

double AngleFitter::getRelativeTime(double timestamp) const {
    // 公有版本，自动加锁
    std::shared_lock<std::shared_mutex> lock(observations_mutex_);
    return getRelativeTimeNoLock(timestamp);
}

double AngleFitter::getRelativeTimeNoLock(double timestamp) const {
    // 使用窗口内第一个观测作为时间基准，而不是全局 start_time_
    // 这样拟合模型始终基于当前窗口数据，避免窗口滚动导致的时间偏移问题
    // 注意：调用者必须已持有 observations_mutex_
    if (!observations_.empty()) {
        return timestamp - observations_.front().timestamp;
    }
    return timestamp - start_time_;
}

void AngleFitter::smoothModelParameters(const AngleModel& new_model) {
    double alpha                 = smooth_alpha_;
    const double one_minus_alpha = 1.0 - alpha;

    if (new_model.is_big_rune == current_model_.is_big_rune) {
        if (new_model.is_big_rune) {
            // 平滑正弦参数
            current_model_.sin_amplitude =
                alpha * new_model.sin_amplitude + one_minus_alpha * current_model_.sin_amplitude;
            current_model_.sin_omega =
                alpha * new_model.sin_omega + one_minus_alpha * current_model_.sin_omega;
            // 【修复】强制 b = 2.090 - a 约束，而不是独立平滑 b
            current_model_.sin_offset = OFFSET_BASE - current_model_.sin_amplitude;
            current_model_.sin_phase  = 0.3 * new_model.sin_phase + 0.7 * current_model_.sin_phase;
            current_model_.sin_const_term =
                alpha * new_model.sin_const_term + one_minus_alpha * current_model_.sin_const_term;
        } else {
            // 平滑线性参数
            current_model_.lin_omega =
                alpha * new_model.lin_omega + one_minus_alpha * current_model_.lin_omega;
            current_model_.lin_offset =
                alpha * new_model.lin_offset + one_minus_alpha * current_model_.lin_offset;
        }
    }
}

double AngleFitter::calculateFitError(const AngleModel& model) {
    std::shared_lock<std::shared_mutex> lock(observations_mutex_);

    double sum_squared_error = 0;
    int count                = 0;

    for (const auto& obs : observations_) {
        double t = getRelativeTimeNoLock(obs.timestamp);
        double predicted;

        if (model.is_big_rune) {
            predicted = -model.sin_amplitude / model.sin_omega
                          * std::cos(model.sin_omega * t + model.sin_phase)
                      + model.sin_offset * t + model.sin_const_term;
        } else {
            predicted = model.lin_omega * t + model.lin_offset;
        }

        double error = obs.continuous_angle - predicted;
        sum_squared_error += error * error;
        count++;
    }

    if (count == 0)
        return 0;
    return std::sqrt(sum_squared_error / count);
}

void AngleFitter::enforceConstraints(AngleModel& model) const {
    // 约束1: a ∈ [0.780, 1.045]
    if (model.sin_amplitude < AMPLITUDE_MIN) {
        model.sin_amplitude = AMPLITUDE_MIN;
    } else if (model.sin_amplitude > AMPLITUDE_MAX) {
        model.sin_amplitude = AMPLITUDE_MAX;
    }

    // 约束2: ω ∈ [1.884, 2.000]
    if (model.sin_omega < OMEGA_MIN) {
        model.sin_omega = OMEGA_MIN;
    } else if (model.sin_omega > OMEGA_MAX) {
        model.sin_omega = OMEGA_MAX;
    }

    // 约束3: b = 2.090 - a (强制约束)
    model.sin_offset = OFFSET_BASE - model.sin_amplitude;
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
    double window_start_time = observations_.empty() ? start_time_ : observations_.front().timestamp;

    for (int i = 1; i <= num_synthetic_frames; i++) {
        double synthetic_time = loss_start_time + i * avg_frame_interval;

        // 用拟合模型预测此时刻的角度（使用窗口相对时间）
        double predicted_angle = 0;
        double t = synthetic_time - window_start_time;
        if (current_model_.is_big_rune) {
            double a     = current_model_.sin_amplitude;
            double omega = current_model_.sin_omega;
            double t0    = current_model_.sin_phase;
            double b     = current_model_.sin_offset;
            double c     = current_model_.sin_const_term;
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
// 【新】最小二乘先验拟合角度参数
// ============================================================================

bool AngleFitter::fitAngleModelLSQ() {
    /*
     * 角度模型: θ(t) = -a/ω·cos(ωt + t₀) + b·t + c
     *
     * 三角展开:
     * -a/ω·cos(ωt + t₀) = -a/ω·[cos(t₀)·cos(ωt) - sin(t₀)·sin(ωt)]
     *                    = A·cos(ωt) + B·sin(ωt)
     * 其中:
     *   A = -a/ω·cos(t₀)
     *   B = a/ω·sin(t₀)
     *
     * 线性模型 (固定ω):
     * θ(t) = A·cos(ωt) + B·sin(ωt) + b·t + c
     *
     * 矩阵形式: Θ = H·X, 其中 X = [A, B, b, c]^T
     */

    std::shared_lock<std::shared_mutex> lock(observations_mutex_);

    if (observations_.size() < static_cast<size_t>(min_data_points_)) {
        return false;
    }

    // 提取数据
    std::vector<double> t_values, theta_values;
    for (const auto& obs : observations_) {
        t_values.push_back(getRelativeTimeNoLock(obs.timestamp));
        theta_values.push_back(obs.continuous_angle);
    }

    lock.unlock();

    int n = static_cast<int>(t_values.size());
    if (n < 4) return false;  // 至少需要4个点求解4个参数

    double best_error = std::numeric_limits<double>::max();
    double best_a = AMPLITUDE_INIT;
    double best_omega = OMEGA_INIT;
    double best_t0 = 0.0;
    double best_b = OFFSET_BASE - best_a;
    double best_c = 0.0;

    // 搜索 ω ∈ [1.884, 2.000]，步长0.005
    for (double omega = OMEGA_MIN; omega <= OMEGA_MAX + 1e-6; omega += 0.005) {

        // 构建设计矩阵 H (n×4)
        Eigen::MatrixXd H(n, 4);
        Eigen::VectorXd Theta(n);

        for (int i = 0; i < n; i++) {
            double t = t_values[i];
            H(i, 0) = std::cos(omega * t);  // A 的系数
            H(i, 1) = std::sin(omega * t);  // B 的系数
            H(i, 2) = t;                     // b 的系数
            H(i, 3) = 1.0;                   // c 的系数
            Theta(i) = theta_values[i];
        }

        // 线性最小二乘解: X = (H'H)⁻¹ H'Θ
        Eigen::Matrix4d HtH = H.transpose() * H;
        Eigen::Vector4d HtTheta = H.transpose() * Theta;

        // 使用 LDLT 分解求解（数值稳定）
        Eigen::Vector4d X = HtH.ldlt().solve(HtTheta);

        double A = X(0);
        double B = X(1);
        double b = X(2);
        double c = X(3);

        // 反解 a, t₀
        // a/ω = sqrt(A² + B²)
        // t₀ = atan2(B, -A)
        double a_over_omega = std::sqrt(A * A + B * B);
        double a = a_over_omega * omega;
        double t0 = std::atan2(B, -A);

        // 计算拟合误差
        Eigen::VectorXd residual = Theta - H * X;
        double error = residual.squaredNorm() / n;  // MSE

        if (error < best_error) {
            best_error = error;
            best_a = a;
            best_omega = omega;
            best_t0 = t0;
            best_b = b;
            best_c = c;
        }
    }

    // 保存先验参数到模型
    {
        std::unique_lock<std::shared_mutex> model_lock(model_mutex_);
        current_model_.sin_amplitude = best_a;
        current_model_.sin_omega = best_omega;
        current_model_.sin_phase = best_t0;
        current_model_.sin_offset = best_b;
        current_model_.sin_const_term = best_c;
        current_model_.fit_error = std::sqrt(best_error);  // RMSE
        current_model_.is_big_rune = true;
        current_model_.is_valid = true;
        current_model_.data_points_used = n;
        current_model_.last_fit_time = t_values.back() + start_time_;
    }

    return true;
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
    current_model_.sin_amplitude = std::clamp(
        current_model_.sin_amplitude, AMPLITUDE_MIN, AMPLITUDE_MAX);

    // 约束2: ω ∈ [1.884, 2.000]
    current_model_.sin_omega = std::clamp(
        current_model_.sin_omega, OMEGA_MIN, OMEGA_MAX);

    // 约束3: b = 2.090 - a (硬约束)
    current_model_.sin_offset = OFFSET_BASE - current_model_.sin_amplitude;
}

bool AngleFitter::fitLinearModelPublic() {
    AngleModel linear_model;
    if (fitLinearModel(linear_model)) {
        linear_model.is_big_rune = false;

        std::unique_lock<std::shared_mutex> lock(model_mutex_);
        // 保留一些状态
        linear_model.last_continuous_angle = current_model_.last_continuous_angle;
        linear_model.last_observation_time = current_model_.last_observation_time;
        linear_model.filtered_velocity = current_model_.filtered_velocity;
        linear_model.filtered_acceleration = current_model_.filtered_acceleration;
        linear_model.velocity_filter_valid = current_model_.velocity_filter_valid;
        linear_model.is_valid = true;
        linear_model.data_points_used = current_model_.data_points_used;

        current_model_ = linear_model;
        return true;
    }
    return false;
}

} // namespace energy_meter
