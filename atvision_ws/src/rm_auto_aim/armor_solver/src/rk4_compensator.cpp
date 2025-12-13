// RK4-based ballistic compensator implementation
#include "armor_solver/rk4_compensator.hpp"

#include <algorithm>

namespace rm_auto_aim {

std::pair<double, double> Rk4Compensator::simulateToX(double x, double angle) const noexcept {
    const double target_x = std::max(0.0, x);
    if (target_x == 0.0 || velocity < 1e-6) {
        return {0.0, 0.0};
    }

    const double dt      = dt_ > 0.0 ? dt_ : 0.001;
    const double t_limit = t_max_ > dt ? t_max_ : dt * 2.0;

    BallisticParams params;
    params.g = gravity;
    params.k = resistance;

    State s{};
    s.x  = 0.0;
    s.y  = 0.0;
    s.vx = velocity * std::cos(angle);
    s.vy = velocity * std::sin(angle);
    if (s.vx <= 1e-6) {
        return {0.0, 0.0};
    }

    double t     = 0.0;
    State prev   = s;
    double t_prev = 0.0;

    const int max_steps = static_cast<int>(t_limit / dt) + 2;
    for (int i = 0; i < max_steps; ++i) {
        prev   = s;
        t_prev = t;

        s = rk4_step(s, dt, params);
        t += dt;

        const bool reached_x = prev.x <= target_x && s.x >= target_x;
        if (reached_x || s.vx <= 1e-6) {
            const double denom = s.x - prev.x;
            double alpha       = denom != 0.0 ? (target_x - prev.x) / denom : 0.0;
            alpha              = std::clamp(alpha, 0.0, 1.0);
            const double yi    = prev.y + alpha * (s.y - prev.y);
            const double ti    = t_prev + alpha * dt;
            return {yi, ti};
        }

        if (s.y < -50.0) {
            // Already well below the muzzle; no need to keep integrating
            return {s.y, t};
        }
    }

    return {s.y, t};
}

double Rk4Compensator::calculateTrajectory(const double x, const double angle) const noexcept {
    return simulateToX(x, angle).first;
}

double Rk4Compensator::getFlyingTime(const Eigen::Vector3d& target_position) const noexcept {
    const double distance =
        std::sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
    if (distance < 1e-6 || velocity < 1e-6) {
        return 0.0;
    }

    double pitch       = std::atan2(target_position(2), distance);
    double tuned_pitch = pitch;
    if (compensate(target_position, tuned_pitch)) {
        pitch = tuned_pitch;
    }

    return simulateToX(distance, pitch).second;
}

} // namespace fyt::auto_aim

