#pragma once
#include <vector>
#include <cmath>
#include <algorithm>

struct BallisticParams {
    double g = 9.80665;     // m/s^2
    // 二次阻力：a_drag = -k * |v| * v
    // k = 0.5 * rho * CdA / m
    double k = 0.0;         // 设为 0 表示无空气阻力（纯抛体）
};

struct Sample {
    double t;   // s
    double x;   // m
    double y;   // m
    double vx;  // m/s
    double vy;  // m/s
};

struct State {
    double x, y, vx, vy;
};

static inline State deriv(const State& s, const BallisticParams& p) {
    const double v2 = s.vx * s.vx + s.vy * s.vy;
    const double v  = std::sqrt(std::max(0.0, v2));

    // dx/dt = vx, dy/dt = vy
    // dv/dt = gravity + drag
    State ds{};
    ds.x  = s.vx;
    ds.y  = s.vy;

    // drag accel = -k * |v| * v_vec
    const double ax_drag = -p.k * v * s.vx;
    const double ay_drag = -p.k * v * s.vy;

    ds.vx = ax_drag;
    ds.vy = -p.g + ay_drag;
    return ds;
}

static inline State rk4_step(const State& s, double dt, const BallisticParams& p) {
    const State k1 = deriv(s, p);

    State s2{ s.x + 0.5*dt*k1.x, s.y + 0.5*dt*k1.y, s.vx + 0.5*dt*k1.vx, s.vy + 0.5*dt*k1.vy };
    const State k2 = deriv(s2, p);

    State s3{ s.x + 0.5*dt*k2.x, s.y + 0.5*dt*k2.y, s.vx + 0.5*dt*k2.vx, s.vy + 0.5*dt*k2.vy };
    const State k3 = deriv(s3, p);

    State s4{ s.x + dt*k3.x, s.y + dt*k3.y, s.vx + dt*k3.vx, s.vy + dt*k3.vy };
    const State k4 = deriv(s4, p);

    State out{};
    out.x  = s.x  + (dt/6.0) * (k1.x  + 2.0*k2.x  + 2.0*k3.x  + k4.x);
    out.y  = s.y  + (dt/6.0) * (k1.y  + 2.0*k2.y  + 2.0*k3.y  + k4.y);
    out.vx = s.vx + (dt/6.0) * (k1.vx + 2.0*k2.vx + 2.0*k3.vx + k4.vx);
    out.vy = s.vy + (dt/6.0) * (k1.vy + 2.0*k2.vy + 2.0*k3.vy + k4.vy);
    return out;
}

// 返回从发射到落地(y=0)的采样点（包含落地点插值）
inline std::vector<Sample> simulate_ballistic_rk4(
    double v0,                 // 初速 m/s（例如 12 或 25）
    double launch_angle_rad,   // 发射仰角（弧度）
    double y0,                 // 初始高度 m
    double dt,                 // 积分步长 s（例如 0.001~0.01）
    double t_max,              // 最大仿真时长 s（兜底）
    const BallisticParams& p
) {
    std::vector<Sample> out;
    out.reserve(static_cast<size_t>(t_max / dt) + 2);

    State s{};
    s.x  = 0.0;
    s.y  = y0;
    s.vx = v0 * std::cos(launch_angle_rad);
    s.vy = v0 * std::sin(launch_angle_rad);

    double t = 0.0;
    out.push_back({t, s.x, s.y, s.vx, s.vy});

    State prev = s;
    double t_prev = t;

    while (t < t_max) {
        prev = s;
        t_prev = t;

        s = rk4_step(s, dt, p);
        t += dt;

        // 落地检测：从 y>0 穿到 y<=0
        if (prev.y > 0.0 && s.y <= 0.0) {
            // 线性插值到 y=0（够用且稳定）
            const double alpha = prev.y / (prev.y - s.y); // in (0,1]
            const double ti  = t_prev + alpha * dt;
            const double xi  = prev.x  + alpha * (s.x  - prev.x);
            const double vxi = prev.vx + alpha * (s.vx - prev.vx);
            const double vyi = prev.vy + alpha * (s.vy - prev.vy);
            out.push_back({ti, xi, 0.0, vxi, vyi});
            break;
        }

        out.push_back({t, s.x, s.y, s.vx, s.vy});
    }

    return out;
}

/*
用法示例（同一角度下，初速 12 和 25 两条轨迹）：

BallisticParams p;
p.g = 9.80665;
p.k = 0.0; // 先不考虑阻力；要考虑阻力就设 k = 0.5*rho*CdA/m

auto traj12 = simulate_ballistic_rk4(12.0, 30.0*M_PI/180.0, 0.0, 0.002, 10.0, p);
auto traj25 = simulate_ballistic_rk4(25.0, 30.0*M_PI/180.0, 0.0, 0.002, 10.0, p);

traj.back().x 就是射程，traj.back().t 是飞行时间
*/
