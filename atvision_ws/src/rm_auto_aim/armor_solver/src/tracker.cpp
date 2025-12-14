#include "armor_solver/tracker.hpp"
#include "armor_solver/util.hpp"
#include <algorithm>
#include <cmath>

namespace rm_auto_aim {

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

    auto matched = is_outpost ? match_all_outpost(armors, idx, outpost_ukf->x())
                              : match_all(armors, idx, robot_ukf->x());
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

        z[0] = ia.pose.position.x;
        z[1] = ia.pose.position.y;
        z[2] = ia.pose.position.z;
        double yaw;
        yaw = util::orientation2yaw(ia.pose.orientation);
        if (is_outpost) {
            z[3] = util::unwrap_rad(
                outpost_ukf->x()[util::O_YAW] + idx[i] * 2 * M_PI / armor_num, yaw);
            outpost_model_.armor_id = idx[i];
            this->set_measurement(z, false);
            outpost_ukf->update(z);
        } else {
            z[3] = util::unwrap_rad(
                robot_ukf->x()[util::YAW] + idx[i] * 2 * M_PI / armor_num, yaw); // -pi - pi

            robot_model_.armor_id = idx[i];
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

    // Start fresh for a new target
    robot_ukf.reset();
    outpost_ukf.reset();

    if (is_outpost) {
        double x0 = pos.x;
        double y0 = pos.y;
        double z0, z1, z2;
        z0 = z1 = z2 = pos.z;
        double yaw   = util::orientation2yaw(tgt.pose.orientation);
        OutpostUKF::VecX xp0{x0, y0, yaw, v0, z0, z1, z2};
        OutpostUKF::MatXX p0 = OutpostUKF::MatXX::Identity();
        outpost_ukf.emplace(outpost_model_, xp0, p0);
    } else {
        double x0      = pos.x;
        double y0      = pos.y;
        double z0      = pos.z;
        double yaw     = util::orientation2yaw(tgt.pose.orientation);
        double radius1 = 0.23;
        double radius2 = 0.23;
        double h       = 0;
        RoboUKF::VecX xp0{x0, v0, y0, v0, z0, v0, yaw, v0, radius1, radius2, h};
        RoboUKF::MatXX P0 = RoboUKF::MatXX::Identity();

        robot_ukf.emplace(robot_model_, xp0, P0);
    }
    state_machine(true);
    return tgt.number;
}

std::vector<rm_interfaces::msg::Armor> Tracker::match_all(
    const rm_interfaces::msg::Armors& armors, std::vector<int>& idx, const RoboUKF::VecX& x_pre) {

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

    // 预计算每个观测的量测向量
    std::vector<VecZ> meas_list(n_obs);
    for (int j = 0; j < n_obs; ++j) {
        const auto& a = armors.armors[j];
        VecZ z;
        z[0] = a.pose.position.x;
        z[1] = a.pose.position.y;
        z[2] = a.pose.position.z;
        z[3] = util::orientation2yaw(a.pose.orientation);

        meas_list[j] = z;
    }

    // 给每个 (观测, armor_id) 计算残差 + 代价
    for (int j = 0; j < n_obs; ++j) {
        if (armors.armors[j].number != name)
            continue; // 只匹配目标编号一致的装甲板
        for (int id = 0; id < armors_num; ++id) {
            // 预测该 armor_id 对应的量测
            VecZ z_pred = robot_model_.h(x_pre, id);

            VecZ nu = meas_list[j] - z_pred;

            auto R = robot_model_.R_sqrt(z_pred); // 实际上返回的是对角协方差
            // yaw 残差归一化到 [-pi, pi]
            nu[3] = util::normalize_rad(nu[3]);

            // 这里用量测噪声协方差 R 近似创新协方差 S
            Eigen::Matrix<double, RobotModel::NZ, RobotModel::NZ> Rinv = R.inverse();

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
    const OutpostUKF::VecX& x_pre) {

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

    std::vector<VecZ> meas_list(n_obs);
    // 构建测量（删除 z 轴）
    for (int j = 0; j < n_obs; ++j) {
        const auto& a = armors.armors[j];
        VecZ z;
        z[0]         = a.pose.position.x;
        z[1]         = a.pose.position.y;
        z[2]         = util::orientation2yaw(a.pose.orientation); // yaw 放在第 3 维
        meas_list[j] = z;
    }

    for (int j = 0; j < n_obs; ++j) {
        if (armors.armors[j].number != name)
            continue;

        for (int id = 0; id < armors_num; ++id) {
            VecZ z_pred_full = outpost_model_.h(x_pre, id);

            // 预测 3 维
            Eigen::Vector3d z_pred;
            z_pred << z_pred_full[0], z_pred_full[1], z_pred_full[3];

            // 残差
            Eigen::Vector3d nu;
            nu[0] = meas_list[j][0] - z_pred[0];
            nu[1] = meas_list[j][1] - z_pred[1];
            nu[2] = util::normalize_rad(meas_list[j][2] - z_pred[2]);

            // R 取 3×3
            auto R_full          = outpost_model_.R_sqrt(z_pred_full);
            Eigen::Matrix3d R    = R_full.topLeftCorner<3, 3>();
            Eigen::Matrix3d Rinv = R.inverse();

            double d2 = nu.transpose() * Rinv * nu;

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
            if (robot_ukf) {
                auto& x = *robot_ukf->x_raw_ptr();
                x.setZero();
            }
            if (outpost_ukf) {
                auto& x = *outpost_ukf->x_raw_ptr();
                x.setZero();
            }
            detecting_count_ = 0;
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
            detecting_count_ = 0;

            auto& x = *robot_ukf->x_raw_ptr();
            x.setZero();
            state = IDLE;
            std::cout << "IDLE" << std::endl;
        }
        break;

    case TRACKING:
        if (!found) {
            state = TEMP_LOST;
            std::cout << "TEMP_LOST" << std::endl;
            lost_count_ = 1;                       // 刚丢一帧
        }
        break;

    case TEMP_LOST:
        if (found) {
            lost_count_ = 0;
            state       = TRACKING;                // 找回来了
            std::cout << "TRACKING" << std::endl;
        } else {
            lost_count_++;
            if (lost_count_ > params.lost_thres) { // 丢太久，放弃
                lost_count_      = 0;
                detecting_count_ = 0;
                state            = IDLE;
                std::cout << "IDLE" << std::endl;
            }
        }
        break;

    default: break;
    }
}

} // namespace armor_tracker
