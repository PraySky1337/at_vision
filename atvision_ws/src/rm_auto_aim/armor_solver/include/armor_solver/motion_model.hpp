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
#pragma once

// ceres
#include <ceres/ceres.h>
// project
#include "rm_utils/math/extended_kalman_filter.hpp"
#include "rm_utils/math/unscented_kalman_filter.hpp"
#include "util.hpp"

namespace fyt::auto_aim {

enum class MotionModel {
    CONSTANT_VELOCITY = 0, // Constant velocity
    CONSTANT_ROTATION = 1, // Constant rotation velocity
    CONSTANT_VEL_ROT  = 2  // Constant velocity and rotation velocity
};

// X_N: state dimension, Z_N: measurement dimension
constexpr int X_N = util::STATE_MAX;
constexpr int Z_N = 4;

struct Predict {
    explicit Predict(double dt, MotionModel model = MotionModel::CONSTANT_VEL_ROT)
        : dt(dt)
        , model(model) {}

    template <typename T>
    void operator()(const T x0[X_N], T x1[X_N]) {
        std::memcpy(x1, x0, X_N * sizeof(T));

        // v_xyz
        if (model == MotionModel::CONSTANT_VEL_ROT || model == MotionModel::CONSTANT_VELOCITY) {
            // linear velocity
            x1[util::XC] += x0[util::VX] * dt;
            x1[util::YC] += x0[util::VY] * dt;
            x1[util::ZC] += x0[util::VZ] * dt;
        } else {
            // no velocity
            x1[util::VX] *= 0.;
            x1[util::VY] *= 0.;
            x1[util::VZ] *= 0.;
        }

        // v_yaw
        if (model == MotionModel::CONSTANT_VEL_ROT || model == MotionModel::CONSTANT_ROTATION) {
            // angular velocity
            x1[6] += x0[7] * dt;
        } else {
            // no rotation
            x1[7] *= 0.;
        }
    }

    double dt;
    MotionModel model;
};

struct Measure {
    Measure() = default;
    explicit Measure(int id, int armors_num)
        : current_id(id), armors_num(armors_num) {}
    int current_id = 0;
    int armors_num = 4;

    template <typename T>
    void operator()(const T x[X_N], T z[Z_N]) {
        int id              = current_id;
        double yaw_rad      = x[util::YAW] + id * 2 * M_PI / armors_num;
        Eigen::Vector3d xyz = util::state2armor_xyz(x, id, armors_num);
        z[0]                = xyz[0];
        z[1]                = xyz[1];
        z[2]                = xyz[2];
        z[3]                = yaw_rad;
    }
};

using RobotStateEKF = at::ExtendedKalmanFilter<X_N, Z_N, Predict, Measure>;
using RobotStateUKF = at::UnscentedKalmanFilter<X_N, Z_N, Predict, Measure>;

} // namespace fyt::auto_aim
