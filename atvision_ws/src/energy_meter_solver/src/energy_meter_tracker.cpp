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

    std::vector<int> matched_blade_ids;
    measurement_.fill(0.0);

    if (!energy_ukf.has_value()) {
        return false;
    }

    if (!match_all(target_positions, target_quats, matched_blade_ids, energy_ukf->x())) {
        // 没有任何通过门控的观测，这一帧只做 predict
        state_machine(false);
        return false;
    }

    // 有观测，就认为 found = true
    state_machine(true);

    // 记录当前追踪的靶ID（取第一个匹配的）
    if (!matched_blade_ids.empty()) {
        current_matched_blade_id_ = matched_blade_ids[0];
    }

    for (size_t i = 0; i < matched_blade_ids.size(); ++i) {
        EnergyUKF::VecZ z;
        z[0] = target_positions[i].x();
        z[1] = target_positions[i].y();
        z[2] = target_positions[i].z();

        // 从四元数提取 roll 角
        double observed_roll = orientation2roll(target_quats[i]);

        // 角度连续化处理：关键逻辑！
        z[3] =
            unwrap_rad(energy_ukf->x()[ROLL] + matched_blade_ids[i] * 2 * M_PI / 5, observed_roll);

        energy_model_.armor_id = matched_blade_ids[i];
        this->set_measurement(z);
        energy_ukf->update(z);
    }
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

    // 初始化 UKF
    EnergyMeter::VecX xp0{xc, yc, zc, initial_roll, v0};
    EnergyUKF::MatXX p0 = EnergyUKF::MatXX::Identity();
    energy_ukf.emplace(energy_model_, xp0, p0);

    state_machine(true);
    return true;
}

bool Tracker::match_all(
    const std::vector<Eigen::Vector3d>& target_positions,
    const std::vector<Eigen::Quaterniond>& target_quats, std::vector<int>& matched_blade_ids,
    const EnergyUKF::VecX& x_pre) {

    matched_blade_ids.clear();

    if (target_positions.empty()) {
        return false;
    }

    const int n_obs      = static_cast<int>(target_positions.size());
    const int armors_num = 5;

    // 代价矩阵：观测 j 与 armor_id k 的匹配代价
    std::vector<std::vector<double>> cost(
        n_obs, std::vector<double>(armors_num, std::numeric_limits<double>::infinity()));

    using VecZ = EnergyMeter::VecZ;

    // 预计算每个观测的量测向量
    std::vector<VecZ> meas_list(n_obs);
    for (int j = 0; j < n_obs; ++j) {
        VecZ z;
        z[0] = target_positions[j].x();
        z[1] = target_positions[j].y();
        z[2] = target_positions[j].z();
        z[3] = orientation2roll(target_quats[j]);

        meas_list[j] = z;
    }

    // 给每个 (观测, armor_id) 计算残差 + 代价
    for (int j = 0; j < n_obs; ++j) {
        for (int id = 0; id < armors_num; ++id) {
            // 预测该 armor_id 对应的量测
            VecZ z_pred = energy_model_.h(x_pre, id);

            VecZ nu = meas_list[j] - z_pred;

            auto R = energy_model_.R_sqrt(z_pred); // 实际上返回的是对角协方差
            // yaw 残差归一化到 [-pi, pi]
            nu[3] = normalize_rad(nu[3]);

            // 这里用量测噪声协方差 R 近似创新协方差 S
            Eigen::Matrix<double, EnergyMeter::NZ, EnergyMeter::NZ> Rinv = R.inverse();

            double d2 = (nu.transpose() * Rinv * nu)(0, 0);

            // 门控
            if (std::isfinite(d2) && d2 < params.matcher_gate) {
                cost[j][id] = d2;
            }
        }
    }

    // 一对一贪心分配：每个观测最多配一个 armor_id，每个 armor_id 只用一次
    std::vector<bool> used_obs(n_obs, false);
    std::vector<bool> used_id(armors_num, false);

    while (true) {
        double best = std::numeric_limits<double>::infinity();
        int best_j  = -1;
        int best_id = -1;

        for (int j = 0; j < n_obs; ++j) {
            if (used_obs[j])
                continue;
            for (int id = 0; id < armors_num; ++id) {
                if (used_id[id])
                    continue;
                if (cost[j][id] < best) {
                    best    = cost[j][id];
                    best_j  = j;
                    best_id = id;
                }
            }
        }

        if (best_j < 0 || best_id < 0) {
            break;                      // 没有更多合格的匹配
        }

        used_obs[best_j] = true;
        used_id[best_id] = true;

        matched_blade_ids.push_back(best_id);
    }

    return !matched_blade_ids.empty();
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
