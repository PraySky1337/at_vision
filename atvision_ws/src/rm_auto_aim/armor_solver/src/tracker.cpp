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
    measurement_.fill(0.0);

    const bool is_outpost = name == "outpost";
    const int armors_num = util::armors_num(name);

    if (is_outpost && !outpost_ukf) {
        return false;
    }

    if (!is_outpost && !robot_ukf) {
        return false;
    }

    // Use template-based DataAssociator (eliminates code duplication)
    typename DataAssociator<RobotModel>::MatchResult match_result;
    std::vector<int> idx;

    if (is_outpost) {
        auto result = outpost_associator_.match(
            armors, outpost_ukf->x(), outpost_ukf->Sx(),
            outpost_model_, name, params.outpost_matcher_gate, armors_num);
        // Convert to compatible format
        for (size_t i = 0; i < result.armors.size(); ++i) {
            match_result.armors.push_back(result.armors[i]);
            idx.push_back(result.armor_ids[i]);
        }
    } else {
        auto result = robot_associator_.match(
            armors, robot_ukf->x(), robot_ukf->Sx(),
            robot_model_, name, params.matcher_gate, armors_num);
        match_result.armors = result.armors;
        idx = result.armor_ids;
    }

    if (match_result.armors.empty()) {
        state_machine(false);
        return false;
    }

    state_machine(true);

    for (size_t i = 0; i < match_result.armors.size(); ++i) {
        RoboUKF::VecZ z;
        auto& ia = match_result.armors[i];

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
