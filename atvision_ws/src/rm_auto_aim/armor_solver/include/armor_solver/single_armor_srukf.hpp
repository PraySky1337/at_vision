#pragma once
#include "srukf.hpp"
#include "util.hpp"
#include <Eigen/Dense>

struct CV3D {
    using Scalar            = double;
    static constexpr int NX = util::STATE_MAX;   // [px,vx, py,vy, pz,vz, yaw, v_yaw]
    static constexpr int NZ = util::MEASURE_MAX; // 量测 [px,py,pz,yaw]

    using VecX   = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ   = Eigen::Matrix<Scalar, NZ, 1>;
    int armor_id = 0;

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

    VecZ h(const VecX& x) const {
        VecZ z;
        using namespace util;
        z[3] = x(YAW) + armor_id * M_PI / 2;
        Scalar radius;
        if (armor_id == 1 || armor_id == 3) {
            z[2]   = x(Z0) + x[H];
            radius = x[R_1];
        } else {
            z[2]   = x(Z0);
            radius = x[R_0];
        }
        z[0] = x(XC) - radius * std::cos(x[YAW] + armor_id * M_PI / 2);
        z[1] = x(YC) - radius * std::sin(x[YAW] + armor_id * M_PI / 2);
        return z;
    }

    Eigen::Matrix<Scalar, NX, NX> Q_sqrt(Scalar dt) const {
        using MatX = Eigen::Matrix<Scalar, NX, NX>;
        MatX Q     = MatX::Zero();
        if (dt <= Scalar(0))
            return Q;

        // 原来：100 / 50 / 30 比较大，更偏向信量测
        // 想更信模型可以先缩小 5~10 倍试试
        const Scalar sigma_a_xy  = 20.0; // or 10.0
        const Scalar sigma_a_z   = 10.0;
        const Scalar sigma_a_yaw = 5.0;

        const Scalar dt2 = dt * dt;
        const Scalar dt3 = dt2 * dt;
        const Scalar dt4 = dt2 * dt2;

        auto fill_block = [&](int idx_p, Scalar q) {
            Q(idx_p, idx_p)         = q * dt4 / 4.0;
            Q(idx_p, idx_p + 1)     = q * dt3 / 2.0;
            Q(idx_p + 1, idx_p)     = q * dt3 / 2.0;
            Q(idx_p + 1, idx_p + 1) = q * dt2;
        };

        fill_block(0, sigma_a_xy * sigma_a_xy);
        fill_block(2, sigma_a_xy * sigma_a_xy);
        fill_block(4, sigma_a_z * sigma_a_z);
        fill_block(6, sigma_a_yaw * sigma_a_yaw);

        Eigen::LLT<MatX> llt(Q);
        MatX S = llt.matrixL();
        return S;
    }
    Eigen::Matrix<Scalar, NZ, Eigen::Dynamic> R_sqrt(const VecZ& z) const {
        // 原来都是 0.5
        // 想“更信模型”，可以适当加大
        const Scalar rx   = 1.0;
        const Scalar ry   = 1.0;
        const Scalar rz   = 1.0;
        const Scalar ryaw = 1.0; // 如果 yaw 抖得厉害，可以加到 2.0 甚至更高

        Eigen::Matrix<Scalar, NZ, NZ> Rs = Eigen::Matrix<Scalar, NZ, NZ>::Zero();
        Rs.diagonal() << rx, ry, rz, ryaw;
        return Rs;
    }
};