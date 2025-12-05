#pragma once
#include "srukf.hpp"
#include "util.hpp"
#include <Eigen/Dense>
#include <cmath>

struct CV3D {
    using Scalar            = double;
    static constexpr int NX = util::STATE_MAX;   // [px,vx, py,vy, pz,vz, yaw, v_yaw]
    static constexpr int NZ = util::MEASURE_MAX; // 量测 [px,py,pz,yaw]

    using VecX   = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ   = Eigen::Matrix<Scalar, NZ, 1>;
    int armor_id = 0;
    int armors_num{4};

    // 恒速模型
    VecX f(const VecX& x, Scalar dt) const {
        VecX xn;
        using namespace util;
        // clang-format off
        xn << x(XC) + x(VX)*dt,
              x(VX),
              x(YC) + x(VY)*dt,
              x(VY),
              x(Z0) + x(VZ)*dt,
              x(VZ),
              x(YAW) + x(V_YAW)*dt,
              x(V_YAW),
              x(R_0),
              x(R_1),
              x(H);
        return xn;
        // clang-format on
    }

    VecZ h(const VecX& x) const { return h(x, armor_id); }

    VecZ h(const VecX& x, int id) const {
        VecZ z;
        using namespace util;
        const Scalar angle_step = 2.0 * M_PI / static_cast<Scalar>(armors_num);
        z[3]                    = x(YAW) + id * angle_step;
        Scalar radius;
        if (id == 1 || id == 3) {
            z[2]   = x(Z0) + x[H];
            radius = x[R_1];
        } else {
            z[2]   = x(Z0);
            radius = x[R_0];
        }
        z[0] = x(XC) - radius * std::cos(x[YAW] + id * M_PI / 2);
        z[1] = x(YC) - radius * std::sin(x[YAW] + id * M_PI / 2);
        return z;
    }

    Eigen::Matrix<Scalar, NX, NX> Q_sqrt(Scalar dt) const {
        using MatX = Eigen::Matrix<Scalar, NX, NX>;
        MatX Q     = MatX::Zero();
        if (dt <= Scalar(0))
            return Q;

        const Scalar sigma_a_xy  = 10.0;
        const Scalar sigma_a_z   = 5.0;
        const Scalar sigma_a_yaw = 5.;

        const Scalar dt2 = dt * dt;
        const Scalar dt3 = dt2 * dt;
        const Scalar dt4 = dt2 * dt2;

        auto fill_block = [&](int idx_p, Scalar q) {
            Q(idx_p, idx_p)         = q * dt4 / 4.0;
            Q(idx_p, idx_p + 1)     = q * dt3 / 2.0;
            Q(idx_p + 1, idx_p)     = q * dt3 / 2.0;
            Q(idx_p + 1, idx_p + 1) = q * dt2;
        };

        // 位置速度的 CV 噪声
        fill_block(0, sigma_a_xy * sigma_a_xy);
        fill_block(2, sigma_a_xy * sigma_a_xy);
        fill_block(4, sigma_a_z * sigma_a_z);
        fill_block(6, sigma_a_yaw * sigma_a_yaw);

        // ========= 给 R0 / R1 加随机游走过程噪声 =========
        using namespace util;
        const Scalar sigma_r0 = 0.01; // 半径 R0 每秒变化标准差
        const Scalar sigma_r1 = 0.01; // 半径 R1
        const Scalar sigma_h  = 0.005;
        // 随机游走：Var ≈ q * dt
        Q(R_0, R_0) += sigma_r0 * sigma_r0 * dt;
        Q(R_1, R_1) += sigma_r1 * sigma_r1 * dt;
        Q(H, H) += sigma_h * sigma_h * dt;

        Eigen::LLT<MatX> llt(Q);
        MatX S = llt.matrixL();
        return S;
    }
    Eigen::Matrix<Scalar, NZ, Eigen::Dynamic> R_sqrt(const VecZ& z) const {
        auto ypd = util::xyz2ypd({z[0], z[1], z[2]});
        constexpr double K = 0.43;
        double weight = std::log(0.43 * ypd[2] + 1.0);
        const Scalar ryaw                = 1.0;
        Eigen::Matrix<Scalar, NZ, NZ> Rs = Eigen::Matrix<Scalar, NZ, NZ>::Zero();
        Rs.diagonal() << weight, weight, weight, ryaw;
        return Rs;
    }
};
