#include "armor_solver/target.hpp"
#include "armor_solver/util.hpp"
#include <cmath>

namespace armor_tracker {

void Tracker::update(const rm_interfaces::msg::Armors& armors) {
    if (!ukf.has_value()) {
        // 外面自己先调用 first_meet_u 初始化
        return;
    }

    std::vector<int> idx;
    auto matched = match_all(armors, idx, ukf->x());
    if (matched.empty()) {
        // 没有任何通过门控的观测，这一帧可以只做 predict（在外面根据时间戳调用）
        state_machine(false);
        return;
    }

    // 有观测，就认为 found = true
    state_machine(true);
    if (matched.size() > 1) {
        RCLCPP_INFO(rclcpp::get_logger("armor_solver"), "update 2 armors!");
    }

    for (size_t i = 0; i < matched.size(); ++i) {
        ImplUKF::VecZ z;
        auto& ia = matched[i];

        z[0] = ia.pose.position.x;
        z[1] = ia.pose.position.y;
        z[2] = ia.pose.position.z;
        double yaw;
        yaw  = util::orientation2yaw(ia.pose.orientation);
        z[3] = util::unwrap_rad(ukf->x()[util::YAW] + idx[i] * 2 * M_PI / armor_num, yaw); // -pi - pi

        cv3d_model_.armor_id = idx[i];
        ukf->update(z);
    }
}

void Tracker::step(double dts, const rm_interfaces::msg::Armors& armors) {
    this->predict(dts);
    this->update(armors);
}

void Tracker::predict(double dts) {
    if (ukf.has_value())
        ukf->predict(dts);
}

rm_interfaces::msg::Target Tracker::get_target() {
    rm_interfaces::msg::Target msg;
    if (ukf.has_value() == false) {
        return msg;
    }
    auto& x = ukf->x();
    using namespace util;
    msg.position.x = x[XC];
    msg.position.y = x[YC];
    msg.position.z = x[Z0];
    msg.velocity.x = x[VX];
    msg.velocity.y = x[VY];
    msg.velocity.z = x[VZ];
    msg.yaw        = x[YAW];
    msg.v_yaw      = x[V_YAW];
    msg.id         = name;
    msg.armors_num = name == "outpost" ? 3 : 4;
    msg.z1         = x[Z0] + x[H];
    msg.radius0    = x[R_0];
    msg.radius1    = x[R_1];
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
    auto& tgt      = armors.armors[min_distance_idx];
    auto& pos      = tgt.pose.position;
    name           = tgt.number;
    armor_num = tgt.number == "outpost" ? 3 : 4;
    double x0      = pos.x;
    double y0      = pos.y;
    double z0      = pos.z;
    double v0      = 0;
    double yaw     = util::orientation2yaw(tgt.pose.orientation);
    double radius1 = 0.26;
    double radius2 = 0.26;
    double h       = 0;
    ImplUKF::VecX xp0{x0, v0, y0, v0, z0, v0, yaw, v0, radius1, radius2, h};
    ImplUKF::MatXX P0 = ImplUKF::MatXX::Identity();

    ukf.emplace(cv3d_model_, xp0, P0);
    state_machine(true);
    return tgt.number;
}

std::vector<rm_interfaces::msg::Armor> Tracker::match_all(
    const rm_interfaces::msg::Armors& armors, std::vector<int>& idx, const ImplUKF::VecX& x_pre) {

    std::vector<rm_interfaces::msg::Armor> matched;
    idx.clear();
    if (state == DETECTING) {
        for (int i = 0; i < armors.armors.size(); i++) {
            auto& ia = armors.armors[i];
            if (ia.number == name) {
                idx.push_back(i);
                break;
            } else {
                continue;
            }
        }
    }

    // 没观测，直接返回空
    if (armors.armors.empty()) {
        return matched;
    }

    const int n_obs      = static_cast<int>(armors.armors.size());
    const int armors_num = (name == "outpost") ? 3 : 4;

    // 简单门控阈值（可以后面自己调）
    constexpr double GATE = 25.0;

    // 代价矩阵：观测 j 与 armor_id k 的匹配代价
    std::vector<std::vector<double>> cost(
        n_obs, std::vector<double>(armors_num, std::numeric_limits<double>::infinity()));

    using VecZ = CV3D::VecZ;

    // 预计算每个观测的量测向量
    std::vector<VecZ> meas_list(n_obs);
    for (int j = 0; j < n_obs; ++j) {
        const auto& a = armors.armors[j];
        VecZ z;
        z[0]         = a.pose.position.x;
        z[1]         = a.pose.position.y;
        z[2]         = a.pose.position.z;
        z[3]         = util::orientation2yaw(a.pose.orientation);
        meas_list[j] = z;
    }

    // 给每个 (观测, armor_id) 计算残差 + 代价
    for (int j = 0; j < n_obs; ++j) {
        for (int id = 0; id < armors_num; ++id) {
            CV3D mdl     = cv3d_model_; // 拷贝一份，避免改到成员
            mdl.armor_id = id;

            // 预测该 armor_id 对应的量测
            VecZ z_pred = mdl.h(x_pre);

            VecZ nu = meas_list[j] - z_pred;

            // yaw 残差归一化到 [-pi, pi]
            while (nu[3] > M_PI)
                nu[3] -= 2.0 * M_PI;
            while (nu[3] < -M_PI)
                nu[3] += 2.0 * M_PI;

            // 这里用量测噪声协方差 R 近似创新协方差 S
            auto R = mdl.R_sqrt(z_pred); // 实际上返回的是对角协方差
            Eigen::Matrix<double, CV3D::NZ, CV3D::NZ> Rinv = R.inverse();

            double d2 = (nu.transpose() * Rinv * nu)(0, 0);

            // 门控：太离谱的直接忽略
            if (std::isfinite(d2) && d2 < GATE) {
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

        matched.push_back(armors.armors[best_j]);
        idx.push_back(best_id);
    }

    return matched;
}

void Tracker::state_machine(bool found) {
    switch (state) {
    case IDLE:
        if (found) {
            detecting_count++;
            if (detecting_count > 0) {  // 第一次见到就进 DETECTING
                state = DETECTING;
            }
        } else {
            detecting_count = 0;
        }
        break;

    case DETECTING:
        if (found) {
            detecting_count++;
            if (detecting_count > 10) { // 连续多帧都能看到
                state           = TRACKING;
                detecting_count = 0;
            }
        } else {
            // 检测中丢了，说明不稳定，回到 IDLE 重新来
            detecting_count = 0;
            state           = IDLE;
        }
        break;

    case TRACKING:
        if (!found) {
            state      = TEMP_LOST;
            lost_count = 1;        // 刚丢一帧
        }
        break;

    case TEMP_LOST:
        if (found) {
            lost_count = 0;
            state      = TRACKING; // 找回来了
        } else {
            lost_count++;
            if (lost_count > 20) { // 丢太久，放弃
                lost_count      = 0;
                detecting_count = 0;
                state           = IDLE;
            }
        }
        break;

    default: break;
    }
}

} // namespace armor_tracker
