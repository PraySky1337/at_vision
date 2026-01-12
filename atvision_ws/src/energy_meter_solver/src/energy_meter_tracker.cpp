#include "energy_meter_solver/energy_meter_tracker.hpp"
#include "energy_meter_solver/types.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace energy_meter {

bool Tracker::update(
    const std::vector<Eigen::Vector3d>& target_positions,
    const std::vector<Eigen::Quaterniond>& target_quats) {

    measurement_.fill(0.0);

    if (!energy_ukf.has_value() || target_positions.empty()) {
        state_machine(false);
        return false;
    }

    // 只处理第一个靶（rune_detector 已保证 targets[0] 是距离上一帧最近的）
    const auto& pos  = target_positions[0];
    const auto& quat = target_quats[0];

    // 为第一个靶找到最佳匹配的 blade_id
    using VecZ = EnergyMeter::VecZ;
    VecZ meas;
    meas[0] = pos.x();
    meas[1] = pos.y();
    meas[2] = pos.z();
    meas[3] = orientation2roll(quat);

    const auto& x_pre    = energy_ukf->x();
    const int armors_num = 5;

    double best_cost = std::numeric_limits<double>::max();
    int best_id      = -1;

    for (int id = 0; id < armors_num; ++id) {
        VecZ z_pred = energy_model_.h(x_pre, id);
        VecZ nu     = meas - z_pred;
        nu[3]       = normalize_rad(nu[3]);

        auto R                                                       = energy_model_.R_sqrt(z_pred);
        Eigen::Matrix<double, EnergyMeter::NZ, EnergyMeter::NZ> Rinv = R.inverse();
        double d2 = (nu.transpose() * Rinv * nu)(0, 0);

        if (std::isfinite(d2) && d2 < params.matcher_gate && d2 < best_cost) {
            best_cost = d2;
            best_id   = id;
        }
    }

    // 门控检查：没有合格匹配
    if (best_id < 0) {
        state_machine(false);
        return false;
    }

    // 有观测，更新状态机
    state_machine(true);
    current_matched_blade_id_ = best_id;

    // 构造量测向量并更新 UKF
    EnergyUKF::VecZ z;
    z[0] = pos.x();
    z[1] = pos.y();
    z[2] = pos.z();
    z[3] = unwrap_rad(x_pre[ROLL] + best_id * 2 * M_PI / 5, meas[3]);

    energy_model_.armor_id = best_id;
    this->set_measurement(z);
    energy_ukf->update(z);

    return true;
}

void Tracker::step(
    double dts, const std::vector<Eigen::Vector3d>& target_positions,
    const std::vector<Eigen::Quaterniond>& target_quats) {
    this->predict(dts);
    this->update(target_positions, target_quats);
}

void Tracker::set_measurement(const EnergyUKF::VecZ& z) {
    measurement_[0] = z[0];
    measurement_[1] = z[1];
    measurement_[2] = z[2];
    measurement_[3] = z[3];
}

void Tracker::predict(double dts) {
    if (energy_ukf.has_value()) {
        energy_ukf->predict(dts);
    }
}

bool Tracker::first_meet_u(
    const Eigen::Vector3d& r_center, const Eigen::Vector3d& target_pos,
    const Eigen::Quaterniond& target_quat) {

    // 从四元数提取初始 roll 角
    double initial_roll = orientation2roll(target_quat);

    double xc = r_center.x();
    double yc = r_center.y();
    double zc = r_center.z();
    double v0 = 0.0;

    // 计算实际半径（R中心到靶心的距离）
    double observed_radius = (target_pos - r_center).norm();
    if (observed_radius > 0.1) {
        energy_model_.params.radius = observed_radius;
    }
    // 否则保持默认值 0.7

    // 初始化 UKF
    EnergyMeter::VecX xp0{xc, yc, zc, initial_roll, v0};
    EnergyUKF::MatXX p0 = EnergyUKF::MatXX::Identity();
    energy_ukf.emplace(energy_model_, xp0, p0);

    state_machine(true);
    return true;
}

void Tracker::state_machine(bool found) {
    switch (state) {
    case IDLE:
        if (found) {
            detecting_count_++;
            if (detecting_count_ > 0) { // 第一次见到就进 DETECTING
                state = DETECTING;
                std::cout << "IDLE -> DETECTING" << std::endl;
            }
        } else {
            // IDLE状态下丢失，完全重置UKF（下次会重新初始化）
            energy_ukf.reset();
            detecting_count_ = 0;
        }
        break;

    case DETECTING:
        if (found) {
            detecting_count_++;
            if (detecting_count_ > params.tracking_thres) { // 连续多帧都能看到
                state = TRACKING;
                std::cout << "DETECTING -> TRACKING" << std::endl;
                detecting_count_ = 0;
            }
        } else {
            // 检测中丢了，说明不稳定，回到 IDLE 重新来
            detecting_count_ = 0;
            energy_ukf.reset();                    // 完全重置，下次重新初始化
            state = IDLE;
            std::cout << "DETECTING -> IDLE" << std::endl;
        }
        break;

    case TRACKING:
        if (!found) {
            state = TEMP_LOST;
            std::cout << "TRACKING -> TEMP_LOST" << std::endl;
            lost_count_ = 1;                       // 刚丢一帧
        }
        break;

    case TEMP_LOST:
        if (found) {
            lost_count_ = 0;
            state       = TRACKING;                // 找回来了
            std::cout << "TEMP_LOST -> TRACKING" << std::endl;
        } else {
            lost_count_++;
            if (lost_count_ > params.lost_thres) { // 丢太久，放弃
                lost_count_      = 0;
                detecting_count_ = 0;
                energy_ukf.reset();                // 完全重置
                state = IDLE;
                std::cout << "TEMP_LOST -> IDLE (lost too long)" << std::endl;
            }
        }
        break;

    default: break;
    }
}

} // namespace energy_meter
