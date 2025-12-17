#include "armor_solver/tracker.hpp"
#include "armor_solver/util.hpp"
#include <algorithm>
#include <cmath>

namespace rm_auto_aim {

void Tracker::reset_tracker_() {
    robot_ukf.reset();
    outpost_ukf.reset();
    name.clear();
    armor_num = 0;
    measurement_.fill(0.0);
    detecting_count_ = 0;
    last_dt_         = 0.0;
    lost_time_       = 0.0;
    state            = IDLE;
}

bool Tracker::update(const rm_interfaces::msg::Armors& armors) {
    std::vector<int> idx;
    const bool is_outpost = name == "outpost";

    measurement_.fill(0.0);

    if (is_outpost && !outpost_ukf) {
        return false;
    }

    if (!is_outpost && !robot_ukf) {
        return false;
    }

    auto matched = is_outpost ? match_all_outpost(armors, idx, outpost_ukf->x(), outpost_ukf->Sx())
                              : match_all(armors, idx, robot_ukf->x(), robot_ukf->Sx());

    if (matched.empty()) {
        // 没有任何通过门控的观测，这一帧只做 predict
        state_machine(false);
        return false;
    }

    // 有观测，就认为 found = true
    state_machine(true);
    // if (matched.size() > 1) {
    //     RCLCPP_INFO(rclcpp::get_logger("armor_solver"), "update 2 armors!");
    // }

    for (size_t i = 0; i < matched.size(); ++i) {
        RoboUKF::VecZ z;
        auto& ia = matched[i];

        const auto& p = ia.pose.position;
        const auto ypd = util::xyz2ypd({p.x, p.y, p.z});
        const double armor_yaw_raw = util::orientation2yaw(ia.pose.orientation);
        if (is_outpost) {
            outpost_model_.armor_id = idx[i];
            const auto z_pred       = outpost_model_.h(outpost_ukf->x(), idx[i]);
            z[0]                    = util::unwrap_rad(z_pred[0], ypd[0]);
            z[1]                    = util::unwrap_rad(z_pred[1], ypd[1]);
            z[2]                    = ypd[2];
            z[3]                    = util::unwrap_rad(z_pred[3], armor_yaw_raw);
            this->set_measurement(z, false);
            outpost_ukf->update(z);
        } else {
            robot_model_.armor_id = idx[i];
            const auto z_pred     = robot_model_.h(robot_ukf->x(), idx[i]);
            z[0]                  = util::unwrap_rad(z_pred[0], ypd[0]);
            z[1]                  = util::unwrap_rad(z_pred[1], ypd[1]);
            z[2]                  = ypd[2];
            z[3]                  = util::unwrap_rad(z_pred[3], armor_yaw_raw);
            bool is_another_pair  = (idx[i] == 1 || idx[i] == 3);
            this->set_measurement(z, is_another_pair);
            robot_ukf->update(z);
        }
    }
    return true;
}

void Tracker::step(double dts, const rm_interfaces::msg::Armors& armors) {
    this->predict(dts);
    this->update(armors);
}

void Tracker::set_measurement(const RoboUKF::VecZ& z, bool is_another_pair) {
    if (is_another_pair) {
        measurement_[4] = z[0];
        measurement_[5] = z[1];
        measurement_[6] = z[2];
        measurement_[7] = z[3];
    } else {
        measurement_[0] = z[0];
        measurement_[1] = z[1];
        measurement_[2] = z[2];
        measurement_[3] = z[3];
    }
}

void Tracker::predict(double dts) {
    last_dt_ = std::max(0.0, dts);
    if (state == TEMP_LOST && (lost_time_ + last_dt_) >= params.lost_thres) {
        reset_tracker_();
        std::cout << "IDLE" << std::endl;
        return;
    }
    if (robot_ukf.has_value()) {
        robot_ukf->predict(dts);
        auto& x = *robot_ukf->x_raw_ptr();
        // 比较粗鲁的防止发散的策略
        x[util::R_0] = std::clamp(x[util::R_0], 0.17, 0.27);
        x[util::R_1] = std::clamp(x[util::R_1], 0.17, 0.27);
        x[util::H]   = std::clamp(x[util::H], -0.2, 0.2);
    }

    if (outpost_ukf.has_value()) {
        outpost_ukf->predict(dts);
        auto& x = *outpost_ukf->x_raw_ptr();
        // if (x[util::O_VYAW] < -0.3) {
        //     x[util::O_VYAW] = -2.51;
        // } else if (x[util::O_VYAW] > 0.3) {
        //     x[util::O_VYAW] = 2.51;
        // }
    }
}

rm_interfaces::msg::Target Tracker::get_target() {
    rm_interfaces::msg::Target msg;
    using namespace util;
    if (robot_ukf) {
        const auto& x  = robot_ukf->x();
        msg.position.x = x[XC];
        msg.position.y = x[YC];
        msg.position.z = x[Z0];
        msg.velocity.x = x[VX];
        msg.velocity.y = x[VY];
        msg.velocity.z = x[VZ];
        msg.yaw        = x[YAW];
        msg.v_yaw      = x[V_YAW];
        msg.id         = name;
        msg.armors_num = 4;
        msg.z1         = x[Z0] + x[H];
        msg.radius0    = x[R_0];
        msg.radius1    = x[R_1];
        msg.tracking   = state == TRACKING || state == TEMP_LOST;
    } else if (outpost_ukf) {
        const auto& x  = outpost_ukf->x();
        msg.position.x = x[O_XC];
        msg.position.y = x[O_YC];
        msg.position.z = x[O_Z0];
        msg.velocity.x = 0.0;
        msg.velocity.y = 0.0;
        msg.velocity.z = 0.0;
        msg.yaw        = x[O_YAW];
        msg.v_yaw      = x[O_VYAW];
        msg.id         = name;
        msg.z1         = x[O_Z1];
        msg.z2         = x[O_Z2];
        msg.armors_num = 3;
        msg.radius0    = OutpostModel::OUTPOST_RADIUS;
        msg.radius1    = OutpostModel::OUTPOST_RADIUS;
        msg.tracking   = state == TRACKING || state == TEMP_LOST;
    }
    return msg;
}

std::string Tracker::first_meet_u(const rm_interfaces::msg::Armors& armors) {
    if (armors.armors.empty()) {
        state_machine(false);
        return {};
    }

    double min_pixel_distance = 1e8;
    int min_distance_idx      = -1;
    for (int i = 0; (size_t)i < armors.armors.size(); i++) {
        auto dist = armors.armors[i].distance_to_image_center;
        if (min_pixel_distance > dist) {
            min_pixel_distance = dist;
            min_distance_idx   = i;
        }
    }
    if (min_distance_idx < 0) {
        state_machine(false);
        return {};
    }
    auto& tgt = armors.armors[min_distance_idx];

    auto& pos = tgt.pose.position;

    name            = tgt.number;
    armor_num       = util::armors_num(name);
    bool is_outpost = name == "outpost";
    double v0       = 0;
    const double armor_yaw = util::orientation2yaw(tgt.pose.orientation);

    // Start fresh for a new target
    robot_ukf.reset();
    outpost_ukf.reset();

    if (is_outpost) {
        const double x0 = pos.x + OutpostModel::OUTPOST_RADIUS * std::cos(armor_yaw);
        const double y0 = pos.y + OutpostModel::OUTPOST_RADIUS * std::sin(armor_yaw);
        double z0, z1, z2;
        z0 = z1 = z2 = pos.z;
        OutpostUKF::VecX xp0{x0, y0, armor_yaw, v0, z0, z1, z2};
        OutpostUKF::MatXX p0 = OutpostUKF::MatXX::Identity();
        outpost_ukf.emplace(outpost_model_, xp0, p0);
    } else {
        constexpr double kDefaultRadius = 0.23;
        const double x0                = pos.x + kDefaultRadius * std::cos(armor_yaw);
        const double y0                = pos.y + kDefaultRadius * std::sin(armor_yaw);
        const double z0                = pos.z;
        const double r0                = kDefaultRadius;
        const double r1                = kDefaultRadius;
        const double h                 = 0.0;
        RoboUKF::VecX xp0{x0, v0, y0, v0, z0, v0, armor_yaw, v0, r0, r1, h};
        RoboUKF::MatXX P0 = RoboUKF::MatXX::Identity();

        robot_ukf.emplace(robot_model_, xp0, P0);
    }
    state_machine(true);
    return tgt.number;
}

std::vector<rm_interfaces::msg::Armor> Tracker::match_all(
    const rm_interfaces::msg::Armors& armors, std::vector<int>& idx, const RoboUKF::VecX& x_pre,
    const RoboUKF::MatXX& Sx_pre) {

    std::vector<rm_interfaces::msg::Armor> matched;
    idx.clear();

    if (armors.armors.empty()) {
        return matched;
    }

    const int n_obs      = static_cast<int>(armors.armors.size());
    const int armors_num = util::armors_num(name);

    // 代价矩阵：观测 j 与 armor_id k 的匹配代价
    std::vector<std::vector<double>> cost(
        n_obs, std::vector<double>(armors_num, std::numeric_limits<double>::infinity()));

    using VecZ = RobotModel::VecZ;
    using MatZ = Eigen::Matrix<double, RobotModel::NZ, RobotModel::NZ>;

    // 预计算每个观测的量测向量
    std::vector<VecZ> meas_list(n_obs);
    for (int j = 0; j < n_obs; ++j) {
        const auto& a = armors.armors[j];
        VecZ z;
        const auto& p  = a.pose.position;
        const auto ypd = util::xyz2ypd({p.x, p.y, p.z});
        z[0]           = ypd[0];
        z[1]           = ypd[1];
        z[2]           = ypd[2];
        z[3]           = util::orientation2yaw(a.pose.orientation);

        meas_list[j] = z;
    }

    // 预计算每个 armor_id 的量测预测与协方差（HPH^T），用于马氏距离门控
    struct PredMeas {
        VecZ z_pred;
        MatZ Pzz;
    };
    std::vector<PredMeas> pred_cache(armors_num);
    {
        constexpr int NX = RobotModel::NX;
        const double gamma = std::sqrt(static_cast<double>(NX));
        const double w     = 1.0 / (2.0 * static_cast<double>(NX));
        for (int id = 0; id < armors_num; ++id) {
            const VecZ z0 = robot_model_.h(x_pre, id);

            std::array<VecZ, 2 * NX> z_sig{};
            for (int k = 0; k < NX; ++k) {
                const RoboUKF::VecX xp = x_pre + gamma * Sx_pre.col(k);
                const RoboUKF::VecX xm = x_pre - gamma * Sx_pre.col(k);

                VecZ zp = robot_model_.h(xp, id);
                VecZ zm = robot_model_.h(xm, id);

                zp[0] = util::unwrap_rad(z0[0], zp[0]);
                zp[1] = util::unwrap_rad(z0[1], zp[1]);
                zp[3] = util::unwrap_rad(z0[3], zp[3]);

                zm[0] = util::unwrap_rad(z0[0], zm[0]);
                zm[1] = util::unwrap_rad(z0[1], zm[1]);
                zm[3] = util::unwrap_rad(z0[3], zm[3]);

                z_sig[k]      = zp;
                z_sig[NX + k] = zm;
            }

            VecZ z_bar = VecZ::Zero();
            for (const auto& zs : z_sig) {
                z_bar.noalias() += w * zs;
            }

            MatZ Pzz = MatZ::Zero();
            for (const auto& zs : z_sig) {
                VecZ dz;
                dz[0] = util::shortest_rad(z_bar[0], zs[0]);
                dz[1] = zs[1] - z_bar[1];
                dz[2] = zs[2] - z_bar[2];
                dz[3] = util::shortest_rad(z_bar[3], zs[3]);
                Pzz.noalias() += w * (dz * dz.transpose());
            }

            pred_cache[id] = {z_bar, Pzz};
        }
    }

    // 给每个 (观测, armor_id) 计算残差 + 代价
    for (int j = 0; j < n_obs; ++j) {
        if (armors.armors[j].number != name)
            continue; // 只匹配目标编号一致的装甲板
        for (int id = 0; id < armors_num; ++id) {
            const auto& z_pred = pred_cache[id].z_pred;

            VecZ nu;
            nu[0] = util::shortest_rad(z_pred[0], meas_list[j][0]);
            nu[1] = meas_list[j][1] - z_pred[1];
            nu[2] = meas_list[j][2] - z_pred[2];
            nu[3] = util::shortest_rad(z_pred[3], meas_list[j][3]);

            const Eigen::Matrix<double, RobotModel::NZ, 1> R_diag =
                robot_model_.R_diag(meas_list[j]);
            MatZ S = pred_cache[id].Pzz;
            S.diagonal() += R_diag;
            S      = (S + S.transpose()) * 0.5;
            S.diagonal().array() += 1e-9;

            const Eigen::LDLT<MatZ> ldlt(S);
            if (ldlt.info() != Eigen::Success) {
                continue;
            }

            double d2 = nu.transpose() * ldlt.solve(nu);

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
            break; // 没有更多合格的匹配
        }

        used_obs[best_j] = true;
        used_id[best_id] = true;

        matched.push_back(armors.armors[best_j]);
        idx.push_back(best_id);
    }

    return matched;
}

std::vector<rm_interfaces::msg::Armor> Tracker::match_all_outpost(
    const rm_interfaces::msg::Armors& armors, std::vector<int>& idx,
    const OutpostUKF::VecX& x_pre, const OutpostUKF::MatXX& Sx_pre) {

    std::vector<rm_interfaces::msg::Armor> matched;
    idx.clear();

    if (armors.armors.empty()) {
        return matched;
    }

    const int n_obs      = static_cast<int>(armors.armors.size());
    const int armors_num = util::armors_num(name);

    // 代价矩阵：观测 j 与 armor_id k 的匹配代价
    std::vector<std::vector<double>> cost(
        n_obs, std::vector<double>(armors_num, std::numeric_limits<double>::infinity()));

    using VecZ = OutpostModel::VecZ;
    using MatZ = Eigen::Matrix<double, OutpostModel::NZ, OutpostModel::NZ>;

    std::vector<VecZ> meas_list(n_obs);
    // 构建测量（球坐标系）
    for (int j = 0; j < n_obs; ++j) {
        const auto& a = armors.armors[j];
        VecZ z;
        const auto& p  = a.pose.position;
        const auto ypd = util::xyz2ypd({p.x, p.y, p.z});
        z[0]           = ypd[0];
        z[1]           = ypd[1];
        z[2]           = ypd[2];
        z[3]           = util::orientation2yaw(a.pose.orientation);
        meas_list[j]   = z;
    }

    // 预计算每个 armor_id 的量测预测与协方差（HPH^T）
    struct PredMeas {
        VecZ z_pred;
        MatZ Pzz;
    };
    std::vector<PredMeas> pred_cache(armors_num);
    {
        constexpr int NX = OutpostModel::NX;
        const double gamma = std::sqrt(static_cast<double>(NX));
        const double w     = 1.0 / (2.0 * static_cast<double>(NX));
        for (int id = 0; id < armors_num; ++id) {
            const VecZ z0 = outpost_model_.h(x_pre, id);

            std::array<VecZ, 2 * NX> z_sig{};
            for (int k = 0; k < NX; ++k) {
                const OutpostUKF::VecX xp = x_pre + gamma * Sx_pre.col(k);
                const OutpostUKF::VecX xm = x_pre - gamma * Sx_pre.col(k);

                VecZ zp = outpost_model_.h(xp, id);
                VecZ zm = outpost_model_.h(xm, id);

                zp[0] = util::unwrap_rad(z0[0], zp[0]);
                zp[1] = util::unwrap_rad(z0[1], zp[1]);
                zp[3] = util::unwrap_rad(z0[3], zp[3]);

                zm[0] = util::unwrap_rad(z0[0], zm[0]);
                zm[1] = util::unwrap_rad(z0[1], zm[1]);
                zm[3] = util::unwrap_rad(z0[3], zm[3]);

                z_sig[k]      = zp;
                z_sig[NX + k] = zm;
            }

            VecZ z_bar = VecZ::Zero();
            for (const auto& zs : z_sig) {
                z_bar.noalias() += w * zs;
            }

            MatZ Pzz = MatZ::Zero();
            for (const auto& zs : z_sig) {
                VecZ dz;
                dz[0] = util::shortest_rad(z_bar[0], zs[0]);
                dz[1] = zs[1] - z_bar[1];
                dz[2] = zs[2] - z_bar[2];
                dz[3] = util::shortest_rad(z_bar[3], zs[3]);
                Pzz.noalias() += w * (dz * dz.transpose());
            }

            pred_cache[id] = {z_bar, Pzz};
        }
    }

    for (int j = 0; j < n_obs; ++j) {
        if (armors.armors[j].number != name)
            continue;

        for (int id = 0; id < armors_num; ++id) {
            const auto& z_pred = pred_cache[id].z_pred;

            VecZ nu;
            nu[0] = util::shortest_rad(z_pred[0], meas_list[j][0]);
            nu[1] = meas_list[j][1] - z_pred[1];
            nu[2] = meas_list[j][2] - z_pred[2];
            nu[3] = util::shortest_rad(z_pred[3], meas_list[j][3]);

            const Eigen::Matrix<double, OutpostModel::NZ, 1> R_diag =
                outpost_model_.R_diag(meas_list[j]);
            MatZ S = pred_cache[id].Pzz;
            S.diagonal() += R_diag;
            S      = (S + S.transpose()) * 0.5;
            S.diagonal().array() += 1e-9;

            const Eigen::LDLT<MatZ> ldlt(S);
            if (ldlt.info() != Eigen::Success) {
                continue;
            }

            double d2 = nu.transpose() * ldlt.solve(nu);

            if (std::isfinite(d2) && d2 < params.outpost_matcher_gate) {
                cost[j][id] = d2;
            }
        }
    }

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
            break;
        }

        used_obs[best_j] = true;
        used_id[best_id] = true;

        matched.push_back(armors.armors[best_j]);
        idx.push_back(best_id);
    }

    return matched;
}

void Tracker::state_machine(bool found) {
    switch (state) {
    case IDLE:
        if (found) {
            detecting_count_++;
            if (detecting_count_ > 0) {                     // 第一次见到就进 DETECTING
                state = DETECTING;
                std::cout << "IDLE" << std::endl;
            }
        } else {
            reset_tracker_();
        }
        break;

    case DETECTING:
        if (found) {
            detecting_count_++;
            if (detecting_count_ > params.tracking_thres) { // 连续多帧都能看到
                state = TRACKING;
                std::cout << "TRACKING" << std::endl;
                detecting_count_ = 0;
            }
        } else {
            // 检测中丢了，说明不稳定，回到 IDLE 重新来
            reset_tracker_();
            std::cout << "IDLE" << std::endl;
        }
        break;

    case TRACKING:
        if (!found) {
            state = TEMP_LOST;
            std::cout << "TEMP_LOST" << std::endl;
            lost_time_ = last_dt_;
        }
        break;

    case TEMP_LOST:
        if (found) {
            lost_time_ = 0.0;
            state       = TRACKING;                // 找回来了
            std::cout << "TRACKING" << std::endl;
        } else {
            lost_time_ += last_dt_;
            if (lost_time_ >= params.lost_thres) { // 丢太久，放弃
                reset_tracker_();
                std::cout << "IDLE" << std::endl;
            }
        }
        break;

    default: break;
    }
}

} // namespace armor_tracker
