#ifndef ENERGY_METER_SOLVER_ANGLE_FITTER_HPP_
#define ENERGY_METER_SOLVER_ANGLE_FITTER_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <random>
#include <shared_mutex>
#include <thread>
#include <vector>


#include "types.hpp"

namespace energy_meter {

/**
 * @brief 将角度运动模型拟合到观测历史 (异步拟合架构)
 *
 * 支持两种模型:
 *
 * 1. 小能量机关(线性):
 *    θ(t) = ω·t + b，转速恒定为 1/3π rad/s
 *    使用最小二乘法拟合，不实现约束
 *
 * 2. 大能量机关(正弦波):
 *    输入角度: 来自 rune_detector 的 roll 角 
 *    角度模型: θ(t) = -a/ω·cos(ωt + t₀) + b·t + c
 *    转速函数: spd(t) = dθ/dt = a·sin(ωt + t₀) + b
 *
 *    约束条件 (仅大能量机关):
 *      - a ∈ [0.780, 1.045] rad/s (转速幅度范围)
 *      - ω ∈ [1.884, 2.000] rad/s (角频率范围)
 *      - b = 2.090 - a (强制约束，转速偏移)
 *      - 实际转速与目标函数时间误差 ≤ 500ms
 *
 *    数据流 (仅大能量机关):
 *      rune_detector (PnP求解, roll = -angle_diffs[id])
 *      → angle_processor (旋转矩阵提取: θ = atan2(R(2,1), R(2,2)))
 *      → AngleObservation
 *      → AngleFitter (异步约束拟合)
 *      → 预测模型
 *
 * 【改进架构】
 * - 异步拟合线程：后台执行RANSAC和参数优化，不阻塞主线程
 * - 自适应RANSAC：根据内点比例动态调整迭代次数
 * - 两阶段网格搜索：粗搜索→细搜索，减少计算量50%
 * - 统一约束管理：在优化目标函数中集成惩罚项
 * - 动态内存管理：可配置观测窗口大小
 */
class AngleFitter {
public:
    explicit AngleFitter(const EnergyMeterConfig& config);
    ~AngleFitter();

    // 禁止拷贝（包含线程对象）
    AngleFitter(const AngleFitter&)            = delete;
    AngleFitter& operator=(const AngleFitter&) = delete;

    /**
     * @brief 添加新的角度观测并更新拟合模型
     * @param observation 新的角度观测数据
     * @return 如果执行了模型更新则返回true，否则返回false
     */
    bool addObservation(const AngleObservation& observation);

    /**
     * @brief 获取当前拟合的模型（线程安全副本）
     */
    AngleModel getModel() const {
        std::shared_lock<std::shared_mutex> lock(model_mutex_);
        return current_model_;
    }

    /**
     * @brief 重置拟合器状态（清空观测，重新开始）
     */
    void reset();

    /**
     * @brief 获取拟合统计信息 - 使用的数据点数量
     */
    int getDataPointCount() const {
        std::shared_lock<std::shared_mutex> lock(observations_mutex_);
        return observations_.size();
    }

    /**
     * @brief 设置外部滤波器提供的角速度（来自 ISRCKF tracker）
     * @param velocity 滤波后的角速度 (rad/s)
     * @param acceleration 滤波后的角加速度 (rad/s²)
     */
    void setFilteredVelocity(double velocity, double acceleration = 0.0) {
        std::unique_lock<std::shared_mutex> lock(model_mutex_);
        current_model_.filtered_velocity = velocity;
        current_model_.filtered_acceleration = acceleration;
        current_model_.velocity_filter_valid = true;
    }

    /**
     * @brief 【新】最小二乘先验拟合角度参数
     *
     * 模型: θ(t) = -a/ω·cos(ωt + t₀) + b·t + c
     * 方法: 搜索ω + 线性LSQ求解 (A, B, b, c)
     *
     * @return 拟合成功返回true
     */
    bool fitAngleModelLSQ();

    /**
     * @brief 【新】应用后验约束
     *
     * 约束条件:
     *   a ∈ [0.780, 1.045]
     *   ω ∈ [1.884, 2.000]
     *   b = 2.090 - a (硬约束)
     */
    void applyPosteriorConstraints();

    /**
     * @brief 拟合小符线性模型（公有接口）
     * 当大符误差太大时调用此方法切换到小符
     */
    bool fitLinearModelPublic();

private:
    // 配置
    EnergyMeterConfig config_;
    int min_data_points_;
    bool auto_detect_type_;
    size_t max_observations_; // 【新】可配置的观测窗口大小

    // 拟合状态（主线程和拟合线程共享，需要同步）
    mutable std::shared_mutex observations_mutex_;
    std::deque<AngleObservation> observations_;

    mutable std::shared_mutex model_mutex_;
    AngleModel current_model_; // 当前拟合的运动模型

    double last_prediction_error_;
    double start_time_;

    // 【新】异步拟合线程相关
    std::thread fitting_thread_;
    std::atomic<bool> stop_thread_{false};
    std::atomic<bool> valid_params_{false};  // 拟合参数有效性标志

    std::mutex fit_signal_mutex_;
    std::condition_variable fit_signal_;     // 通知拟合线程有新数据
    std::atomic<bool> fit_triggered_{false}; // 标记是否触发拟合

    std::atomic<int> ransac_iterations_;     // 【新】RANSAC自适应迭代计数

    // 【新】随机数生成器（RANSAC和采样）
    std::mt19937 random_engine_{std::random_device{}()};

    // 大能量机关参数约束(激活状态)
    static constexpr double AMPLITUDE_MIN  = 0.780;
    static constexpr double AMPLITUDE_MAX  = 1.045;
    static constexpr double AMPLITUDE_INIT = (AMPLITUDE_MIN + AMPLITUDE_MAX) / 2.0;

    static constexpr double OMEGA_MIN  = 1.884;
    static constexpr double OMEGA_MAX  = 2.000;
    static constexpr double OMEGA_INIT = (OMEGA_MIN + OMEGA_MAX) / 2.0;

    static constexpr double OFFSET_BASE = 2.090;

    // 【新】网格搜索参数（粗→细两阶段）
    static constexpr double GRID_COARSE_STEP_A     = 0.1;  // 粗搜索a步长
    static constexpr double GRID_COARSE_STEP_OMEGA = 0.05; // 粗搜索ω步长
    static constexpr double GRID_FINE_STEP_A       = 0.02; // 细搜索a步长
    static constexpr double GRID_FINE_STEP_OMEGA   = 0.01; // 细搜索ω步长

    // 【新】RANSAC参数
    static constexpr int RANSAC_INITIAL_ITER              = 3;   // 初始迭代次数
    static constexpr int RANSAC_MAX_ITER                  = 5;   // 最大迭代次数
    static constexpr double RANSAC_INLIER_RATIO_THRESHOLD = 0.6; // 内点比例阈值

    // 【新】从配置提取的参数
    double smooth_alpha_; // 参数平滑系数
    int fit_interval_ms_; // 拟合执行间隔（毫秒）

    // 速度相关
    double last_angle_;             // 上一次的角度（用于计算速度）
    double last_timestamp_;         // 上一次的时间戳
    bool has_previous_observation_; // 是否有上一次观测

    // 用于短暂丧失恢复的虚拟数据生成
    double last_real_observation_time_;               // 最后一个真实观测的时间戳
    double last_real_observation_angle_;              // 最后一个真实观测的角度
    std::deque<double> frame_interval_history_;       // 帧间隔历史，用于计算平均帧率
    static constexpr size_t MAX_FRAME_INTERVALS = 20; // 最多记录20个帧间隔


    /**
     * @brief 异步拟合线程的主函数
     * 类似Calculate.cpp中的fit()函数，后台执行RANSAC和参数优化
     */
    void fitThreadMain();

    /**
     * @brief 触发一次拟合（由addObservation调用）
     */
    void triggerFitting();

    /**
     * @brief 执行一次完整的拟合（在拟合线程中调用）
     * 包括RANSAC、参数优化、约束验证
     */
    bool performFitting();

    /**
     * @brief 【改进】使用惩罚项拟合正弦波模型(激活状态)
     * 包括两阶段网格搜索和LM优化
     */
    bool fitSinusoidalModel(AngleModel& model);

    /**
     * @brief 【改进】使用两阶段网格搜索初始化参数
     * 粗搜索→细搜索，减少计算量
     */
    bool twoStageGridSearch(
        const std::vector<double>& t_values, const std::vector<double>& angle_values,
        double& best_a, double& best_omega, double& best_t0, double& best_c);

    /**
     * @brief 【改进】自适应RANSAC离群点剔除
     * 根据内点比例动态调整迭代次数
     */
    std::vector<int> adaptiveRansacOutlierRejection();

    /**
     * @brief RANSAC单次迭代（粗搜索）
     */
    std::vector<int> ransacIterateOnce(
        const std::vector<int>& sample_indices, const std::vector<int>& all_indices,
        double outlier_threshold);

    /**
     * @brief 使用LM优化细化参数
     */
    bool levenbergMarquardtOptimization(
        const std::vector<double>& t_values, const std::vector<double>& angle_values, double& a,
        double& omega, double& t0, double& c);

    /**
     * @brief 【改进】统一的约束检查和强制
     * 所有参数约束在此集中处理
     */
    void enforceConstraints(AngleModel& model) const;

    /**
     * @brief 拟合线性模型: θ(t) = ω·t + b
     * 使用最小二乘法回归
     */
    bool fitLinearModel(AngleModel& model);

    /**
     * @brief 从观测历史中剔除离群点
     */
    std::vector<int> rejectOutliers();

    /**
     * @brief 计算相对时间（公有版本，自动加锁）
     */
    double getRelativeTime(double timestamp) const;

    /**
     * @brief 计算相对时间（无锁版本，调用者必须持有 observations_mutex_）
     * 使用窗口内第一个观测作为时间基准
     */
    double getRelativeTimeNoLock(double timestamp) const;

    /**
     * @brief 使用指数移动平均平滑参数变化
     */
    void smoothModelParameters(const AngleModel& new_model);

    /**
     * @brief 线性回归辅助函数
     */
    static bool linearRegression(
        const std::vector<double>& x_values, const std::vector<double>& y_values, double& slope,
        double& intercept);

    /**
     * @brief 计算模型拟合误差 (RMS误差)
     */
    double calculateFitError(const AngleModel& model);

    /**
     * @brief 计算平均帧率
     * @return 平均帧间隔（秒）
     */
    double getAverageFrameInterval() const;

    /**
     * @brief 【新增】生成虚拟观测数据填补丧失期间的空白
     * @param loss_start_time 丧失开始时间
     * @param loss_end_time   恢复时间
     * @param avg_frame_interval 平均帧间隔
     */
    void generateSyntheticObservations(
        double loss_start_time, double loss_end_time, double avg_frame_interval);

}; // class AngleFitter

} // namespace energy_meter

#endif // ENERGY_METER_SOLVER_ANGLE_FITTER_HPP_
