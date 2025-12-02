// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "armor_solver/armor_tracker.hpp"
// std
#include <cfloat>
#include <memory>
#include <string>
// ros2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <angles/angles.h>
// project
#include "armor_solver/util.hpp"
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
    : tracker_state(LOST)
    , tracked_id(std::string(""))
    , measurement(Eigen::VectorXd::Zero(4))
    , target_state(Eigen::VectorXd::Zero(X_N))
    , max_match_distance_(max_match_distance)
    , max_match_yaw_diff_(max_match_yaw_diff)
    , detect_count_(0)
    , lost_count_(0)
    , last_yaw_(0) {}

void Tracker::init(const Armors::SharedPtr& armors_msg) noexcept {
    if (armors_msg->armors.empty()) {
        return;
    }

    // Simply choose the armor that is closest to image center
    double min_distance = DBL_MAX;
    auto& tracked_armor = armors_msg->armors[0];
    for (const auto& armor : armors_msg->armors) {
        if (armor.distance_to_image_center < min_distance) {
            min_distance  = armor.distance_to_image_center;
            tracked_armor = armor;
        }
    }
    const auto& a = tracked_armor;
    update_count  = 0;
    double xa     = a.pose.position.x;
    double ya     = a.pose.position.y;
    double za     = a.pose.position.z;
    double yaw    = orientationToYaw(a.pose.orientation);
    last_yaw_ = yaw;

    // Set initial position at 0.2m behind the target
    target_state = Eigen::VectorXd::Zero(X_N);
    double r     = 0.26;
    double xc    = xa + r * cos(yaw);
    double yc    = ya + r * sin(yaw);
    target_state << xc, 0, yc, 0, za, 0, yaw, 0, r, 0, 0;

    kf->setState(target_state);
    FYT_INFO("armor_solver", "Init EKF!");

    tracked_id    = tracked_armor.number;
    tracker_state = DETECTING;

    if (tracked_id == "outpost") {
        tracked_armors_num = ArmorsNum::OUTPOST_3;
    } else {
        tracked_armors_num = ArmorsNum::NORMAL_4;
    }
}

void Tracker::update(
    Armors::SharedPtr& armors_msg, const armor_tracker::Matcher& matcher) noexcept {
    // 1) UKF 预测一步
    Eigen::VectorXd ekf_prediction = kf->predict();
    ekf_prediction(util::V_YAW)    = std::clamp(ekf_prediction(util::V_YAW), -20.0, 20.0);
    target_state                   = ekf_prediction;

    // 2) 构造 Target，给 matcher 算四个装甲板几何位置
    rm_interfaces::msg::Target target_msg = util::state2target(ekf_prediction);
    target_msg.armors_num                 = (tracked_armors_num == ArmorsNum::NORMAL_4) ? 4 : 3;
    target_msg.id                         = tracked_id;
    target_msg.yaw                        = util::limit_rad(target_msg.yaw);

    // 3) 调 matcher：就地筛掉不属于本目标的装甲板（最多保留 2 个）
    auto matched_pos_ids = matcher.filter(*armors_msg, target_msg);

    bool matched           = false;
    Eigen::VectorXd x_post = ekf_prediction;
    if (!armors_msg->armors.empty()) {
        const int armors_num = target_msg.armors_num > 0
                                   ? target_msg.armors_num
                                   : static_cast<int>(tracked_armors_num);
        for (size_t idx = 0; idx < armors_msg->armors.size(); ++idx) {
            const auto& armor = armors_msg->armors[idx];
            // 如需保险，可再按 id 过滤：
            // if (armor.number != tracked_id) continue;

            const auto& p = armor.pose.position;
            double yaw_meas = orientationToYaw(armor.pose.orientation);

            // 4) 对这一块装甲做一次量测更新
            Eigen::Vector4d measure_x;
            measure_x << p.x, p.y, p.z, yaw_meas;

            int armor_id = 0;
            if (idx < matched_pos_ids.size()) {
                armor_id = matched_pos_ids[idx];
            }
            Measure measure_func{armor_id, armors_num};
            kf->setMeasureFunc(measure_func);

            x_post  = kf->update(measure_x);
            matched = true;
            ++update_count;
        }
    } else {
        FYT_WARN("armor_solver", "No armors after matcher.filter()");
    }

    target_state = x_post;

    // 7) 写回 UKF
    kf->setState(target_state);

    // 8) 状态机：用 matched 这个标志即可
    switch (tracker_state) {
    case DETECTING:
        if (matched) {
            detect_count_++;
            if (detect_count_ > tracking_thres) {
                detect_count_ = 0;
                tracker_state = TRACKING;
                FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
            }
        } else {
            detect_count_ = 0;
            tracker_state = LOST;
            FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
        }
        break;

    case TRACKING:
        if (!matched) {
            tracker_state = TEMP_LOST;
            lost_count_++;
            FYT_DEBUG("armor_solver", "Tracker state: TEMP_LOST {}", tracked_id);
        }
        break;

    case TEMP_LOST:
        if (!matched) {
            lost_count_++;
            if (lost_count_ > lost_thres) {
                lost_count_   = 0;
                tracker_state = LOST;
                FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
            }
        } else {
            tracker_state = TRACKING;
            lost_count_   = 0;
            FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
        }
        break;

    default: break;
    }
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion& q) noexcept {
    // Get armor yaw
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    // Make yaw change continuous (-pi~pi to -inf~inf)
    yaw       = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
    last_yaw_ = yaw;
    return yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd& x) noexcept {
    // Calculate predicted position of the current armor
    double xc = x(0), yc = x(2), za = x(4) + x(9);
    double yaw = x(6), r = x(8);
    double xa = xc - r * cos(yaw);
    double ya = yc - r * sin(yaw);
    return Eigen::Vector3d(xa, ya, za);
}

rm_interfaces::msg::Target Tracker::predict() const {
    rm_interfaces::msg::Target target;
    auto prediction   = kf->predict();
    target            = util::state2target(prediction);
    target.armors_num = tracked_armors_num == ArmorsNum::NORMAL_4 ? 4 : 3;
    return target;
}

} // namespace fyt::auto_aim
