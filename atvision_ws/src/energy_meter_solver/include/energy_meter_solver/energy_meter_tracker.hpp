#pragma once
#include "isrckf.hpp"
#include "motion_model.hpp"
#include <Eigen/Dense>
#include <array>
#include <map>
#include <optional>
#include <rclcpp/time.hpp>
#include <vector>

namespace energy_meter {

struct Tracker {
    struct Params {
        int lost_thres      = 10;
        int tracking_thres  = 3;
        double matcher_gate = 25.0;
        EnergyMeter::Params energy_params;

    } params;
    enum State : uint8_t {
        IDLE,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } state;
    explicit Tracker()
        : state(IDLE)
        , detecting_count_(0)
        , lost_count_(0) {}
    explicit Tracker(const Params& param)
        : Tracker() {
        set_params(param);
    }

    void set_params(const Params& p) {
        params               = p;
        energy_model_.params = params.energy_params;
    }

    using EnergyUKF = srukf::ISRCKF<EnergyMeter, EnergyMeter::NX, EnergyMeter::NZ>;

    std::optional<EnergyUKF> energy_ukf;

    const auto& get_measurement() const noexcept { return measurement_; }

    // 获取 UKF 状态向量
    const EnergyUKF::VecX& get_state() const {
        static EnergyUKF::VecX zero = EnergyUKF::VecX::Zero();
        return energy_ukf.has_value() ? energy_ukf->x() : zero;
    }

    // 获取当前追踪的靶ID（用于预测时的角度偏移）
    int getCurrentBladeId() const { return current_matched_blade_id_; }

    // 初始化跟踪器（直接传原始四元数和位置）
    bool first_meet_u(
        const Eigen::Vector3d& r_center, const Eigen::Vector3d& target_pos,
        const Eigen::Quaterniond& target_quat);

    // 执行预测+更新步骤（直接传四元数，内部计算连续角度和叶片ID）
    void step(
        double dts, const std::vector<Eigen::Vector3d>& target_positions,
        const std::vector<Eigen::Quaterniond>& target_quats);

    // 预测步骤
    void predict(double dts);

    // 更新步骤（传多个观测，内部判断最佳叶片）
    bool update(
        const std::vector<Eigen::Vector3d>& target_positions,
        const std::vector<Eigen::Quaterniond>& target_quats);

private:
    bool match_all(
        const std::vector<Eigen::Vector3d>& target_positions,
        const std::vector<Eigen::Quaterniond>& target_quats, std::vector<int>& matched_blade_ids,
        const EnergyUKF::VecX& x_pre);

    void state_machine(bool found);
    void set_measurement(const EnergyUKF::VecZ& z);
    std::array<double, EnergyMeter::NZ> measurement_;

    // motion_model
    EnergyMeter energy_model_;

    int detecting_count_;
    std::map<int, std::string> voter_;
    int lost_count_;
    int current_matched_blade_id_ = 0;  // 当前追踪的靶ID
};

} // namespace energy_meter
