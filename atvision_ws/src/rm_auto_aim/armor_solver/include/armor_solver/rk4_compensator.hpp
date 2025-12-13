// RK4-based ballistic compensator for armor solver
#pragma once

#include <utility>
#include <cmath>

#include "armor_solver/ballistic.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"

namespace rm_auto_aim {

class Rk4Compensator : public fyt::TrajectoryCompensator {
public:
    explicit Rk4Compensator(double dt = 0.001, double t_max = 8.0) : dt_(dt), t_max_(t_max) {}
    ~Rk4Compensator() override = default;

    double getFlyingTime(const Eigen::Vector3d& target_position) const noexcept override;

protected:
    double calculateTrajectory(double x, double angle) const noexcept override;

private:
    // Integrate until reaching the target horizontal distance; return {y, time}
    std::pair<double, double> simulateToX(double x, double angle) const noexcept;

    double dt_;
    double t_max_;
};

} // namespace fyt::auto_aim

