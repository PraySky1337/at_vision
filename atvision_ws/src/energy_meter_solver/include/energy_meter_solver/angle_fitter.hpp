#ifndef ENERGY_METER_SOLVER_ANGLE_FITTER_HPP_
#define ENERGY_METER_SOLVER_ANGLE_FITTER_HPP_

#include <deque>
#include <mutex>
#include <random>
#include <shared_mutex>
#include <vector>

#include "types.hpp"

namespace energy_meter {

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
    void setFilteredVelocity(double velocity) {
        std::unique_lock<std::shared_mutex> lock(model_mutex_);
        current_model_.filtered_velocity     = velocity;
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
    size_t max_observations_; // 【新】可配置的观测窗口大小

    // 拟合状态（主线程和拟合线程共享，需要同步）
    mutable std::shared_mutex observations_mutex_;
    std::deque<AngleObservation> observations_;

    mutable std::shared_mutex model_mutex_;
    AngleModel current_model_; // 当前拟合的运动模型

    double last_prediction_error_;
    double start_time_;

    // 随机数生成器
    std::mt19937 random_engine_{std::random_device{}()};

    // 大能量机关参数约束(激活状态)
    static constexpr double AMPLITUDE_MIN  = 0.780;
    static constexpr double AMPLITUDE_MAX  = 1.045;
    static constexpr double AMPLITUDE_INIT = (AMPLITUDE_MIN + AMPLITUDE_MAX) / 2.0;

    static constexpr double OMEGA_MIN  = 1.884;
    static constexpr double OMEGA_MAX  = 2.000;
    static constexpr double OMEGA_INIT = (OMEGA_MIN + OMEGA_MAX) / 2.0;

    static constexpr double OFFSET_BASE = 2.090;

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
     * @brief 拟合线性模型: θ(t) = ω·t + b
     * 使用最小二乘法回归
     */
    bool fitLinearModel(AngleModel& model);

    /**
     * @brief 计算相对时间（无锁版本，调用者必须持有 observations_mutex_）
     * 使用窗口内第一个观测作为时间基准
     */
    double getRelativeTimeNoLock(double timestamp) const;

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
