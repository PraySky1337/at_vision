#pragma once
#include "isrckf.hpp"
#include "types.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace energy_meter {

// 能量机关转动roll

// x-c y-c z-c roll vroll

// predict x,y,c ,v*dt

// H 状态量转观测量 01234
//  update之前 传id
// 算出观测靶*id


struct EnergyMeter {
    using Scalar            = double;
    static constexpr int NX = STATE_MAX;
    static constexpr int NZ = MEASURE_MAX; // 量测 [px,py,pz,yaw]

    using VecX = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ = Eigen::Matrix<Scalar, NZ, 1>;

    struct Params {
        // 过程噪声
        Scalar sigma_xy   = 10.0;
        Scalar sigma_z    = 1.0;
        Scalar sigma_roll = 1.0;

        Scalar radius = 0.7;

        // 观测噪声（和距离、yaw 误差有关）
        Scalar meas_dist_k = 0.43; // 距离权重的系数 K
    };

    int armor_id                    = 0;
    static constexpr int ARMORS_NUM = 5;

    Params params;                 // 所有可调参数集中在这里

    EnergyMeter() = default;
    explicit EnergyMeter(const Params& p)
        : params(p) {}

    VecX f(const VecX& x, Scalar dt) const {
        VecX xn;
        // clang-format off
        xn << x(XC) ,
              x(YC),
              x(ZC) ,
              x(ROLL) + x(V_ROLL)*dt,
              x(V_ROLL);
        // clang-format on
        return xn;
    }

    VecZ h(const VecX& x) const { return h(x, armor_id); }

    VecZ h(const VecX& x, int id) const {
        VecZ z;
        assert(id >= 0 && id <= ARMORS_NUM - 1);

        const Scalar angle_step = 2.0 * M_PI / static_cast<Scalar>(ARMORS_NUM);

        double tmp_roll = x(ROLL) + id * angle_step;

        z[3] = tmp_roll;

        Scalar radius = params.radius;

        // 绕X轴旋转的圆周运动模型：
        // 相对位置：x_rel=0, y_rel=R·sin(θ), z_rel=-R·cos(θ)
        // 世界位置：P = R_center + P_rel
        z[0] = x(XC);
        z[1] = x(YC) + radius * std::sin(tmp_roll);
        z[2] = x(ZC) - radius * std::cos(tmp_roll);
        return z;
    }

    Eigen::Matrix<Scalar, NX, NX> Q_sqrt(Scalar dt) const {
        using MatX = Eigen::Matrix<Scalar, NX, NX>;
        MatX Q     = MatX::Zero();
        if (dt <= Scalar(0))
            return Q;

        const Scalar sigma_xy2   = params.sigma_xy * params.sigma_xy;
        const Scalar sigma_z2    = params.sigma_z * params.sigma_z;
        const Scalar sigma_roll2 = params.sigma_roll * params.sigma_roll;

        const Scalar dt2 = dt * dt;
        const Scalar dt3 = dt2 * dt;
        const Scalar dt4 = dt2 * dt2;

        Q(XC, XC) = sigma_xy2 * dt2;
        Q(YC, YC) = sigma_xy2 * dt2;
        Q(ZC, ZC) = sigma_z2 * dt2;

        Q(ROLL, ROLL)     = sigma_roll2 * dt4 / 4.0;
        Q(ROLL, V_ROLL)   = sigma_roll2 * dt3 / 2.0;
        Q(V_ROLL, ROLL)   = Q(ROLL, V_ROLL);
        Q(V_ROLL, V_ROLL) = sigma_roll2 * dt2;

        Eigen::LLT<MatX> llt(Q);
        MatX S = llt.matrixL();
        return S;
    }

    Eigen::Matrix<Scalar, NZ, Eigen::Dynamic> R_sqrt(const VecZ& z) const {
        auto ypd         = xyz2ypd({z[0], z[1], z[2]});
        double delta_yaw = shortest_rad(ypd[0], z[3]);

        const double K = params.meas_dist_k;
        double weight  = std::log(K * ypd[2] + 1.0);

        Eigen::Matrix<Scalar, NZ, NZ> Rs = Eigen::Matrix<Scalar, NZ, NZ>::Zero();
        Rs.diagonal() << weight + std::abs(delta_yaw / 3 + 1), weight + std::abs(delta_yaw / 3 + 1),
            weight, log(std::abs(ypd[2]) + 1) / 200 + 9e-2;
        return Rs;
    }
};

} // namespace energy_meter
