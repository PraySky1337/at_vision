// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef ARMOR_SOLVER_SOLVER_PARAMS_HPP_
#define ARMOR_SOLVER_SOLVER_PARAMS_HPP_

#include <atomic>
#include <string>

namespace rm_auto_aim {

struct TrajectoryParams {
    int iteration_times{20};
    double bullet_speed{25.0};
    double gravity{9.8};
    double resistance{0.001};
};

struct SolverParams {
    double shooting_range_w{0.135};
    double shooting_range_h{0.135};
    double max_tracking_v_yaw{6.0};
    double prediction_delay{0.0};
    double side_angle{15.0};
    double min_switching_v_yaw{1.0};
    int transfer_thresh{5};

    TrajectoryParams trajectory;
    std::string compensator_type{"resistance"};
};

// Thread-safe atomic parameters for hot-reload
struct AtomicSolverParams {
    std::atomic<double> max_tracking_v_yaw{6.0};
    std::atomic<double> prediction_delay{0.0};
    std::atomic<double> side_angle{15.0};
    std::atomic<double> min_switching_v_yaw{1.0};

    void update(const SolverParams& params) {
        max_tracking_v_yaw.store(params.max_tracking_v_yaw, std::memory_order_relaxed);
        prediction_delay.store(params.prediction_delay, std::memory_order_relaxed);
        side_angle.store(params.side_angle, std::memory_order_relaxed);
        min_switching_v_yaw.store(params.min_switching_v_yaw, std::memory_order_relaxed);
    }
};

} // namespace rm_auto_aim

#endif // ARMOR_SOLVER_SOLVER_PARAMS_HPP_
