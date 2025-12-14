#pragma once
#include "srukf.hpp"
#include "util.hpp"
#include <Eigen/Dense>
#include <cmath>

struct RobotModel {
    using Scalar            = double;
    static constexpr int NX = util::STATE_MAX;
    static constexpr int NZ = util::MEASURE_MAX; // 量测 [px,py,pz,yaw]

    using VecX = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ = Eigen::Matrix<Scalar, NZ, 1>;

    struct Params {
        // 过程噪声
        Scalar sigma_a_xy  = 10.0;
        Scalar sigma_a_z   = 1.0;
        Scalar sigma_a_yaw = 1.0;

        Scalar sigma_r0 = 1.0; // 半径 R0 每秒变化标准差
        Scalar sigma_h  = 0.5; // 高度 H

        // 观测噪声（和距离、yaw 误差有关）
        Scalar meas_dist_k = 0.43;  // 距离权重的系数 K
        Scalar yaw_log_k   = 0.005; // yaw 噪声前的系数
    };

    int armor_id                    = 0;
    static constexpr int ARMORS_NUM = 4;

    Params params;                  // 所有可调参数集中在这里

    RobotModel() = default;
    explicit RobotModel(const Params& p)
        : params(p) {}

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
        // clang-format on
        return xn;
    }

    VecZ h(const VecX& x) const { return h(x, armor_id); }

    VecZ h(const VecX& x, int id) const {
        VecZ z;
        using namespace util;
        assert(id >= 0 && id <= ARMORS_NUM - 1);

        const Scalar angle_step = 2.0 * M_PI / static_cast<Scalar>(ARMORS_NUM);

        double tmp_yaw = x(YAW) + id * angle_step;

        z[3] = tmp_yaw;

        Scalar radius;
        if (id == 1 || id == 3) {
            z[2]   = x(Z0) + x[H];
            radius = x[R_1];
        } else {
            z[2]   = x(Z0);
            radius = x[R_0];
        }

        z[0] = x(XC) - radius * std::cos(tmp_yaw);
        z[1] = x(YC) - radius * std::sin(tmp_yaw);
        return z;
    }

    Eigen::Matrix<Scalar, NX, NX> Q_sqrt(Scalar dt) const {
        using MatX = Eigen::Matrix<Scalar, NX, NX>;
        MatX Q     = MatX::Zero();
        if (dt <= Scalar(0))
            return Q;

        const Scalar sigma_a_xy2  = params.sigma_a_xy * params.sigma_a_xy;
        const Scalar sigma_a_z2   = params.sigma_a_z * params.sigma_a_z;
        const Scalar sigma_a_yaw2 = params.sigma_a_yaw * params.sigma_a_yaw;

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
        fill_block(0, sigma_a_xy2);
        fill_block(2, sigma_a_xy2);
        fill_block(4, sigma_a_z2);
        fill_block(6, sigma_a_yaw2);

        // R0 R1 H 过程噪声
        using namespace util;
        const Scalar sigma_r0_2 = params.sigma_r0 * params.sigma_r0;
        const Scalar sigma_r1_2 = params.sigma_r0 * params.sigma_r0;
        const Scalar sigma_h_2  = params.sigma_h * params.sigma_h;

        Q(R_0, R_0) += sigma_r0_2 * dt;
        Q(R_1, R_1) += sigma_r1_2 * dt;
        Q(H, H) += sigma_h_2 * dt;

        Eigen::LLT<MatX> llt(Q);
        MatX S = llt.matrixL();
        return S;
    }

    Eigen::Matrix<Scalar, NZ, Eigen::Dynamic> R_sqrt(const VecZ& z) const {
        auto ypd         = util::xyz2ypd({z[0], z[1], z[2]});
        double delta_yaw = util::shortest_deg(ypd[0], z[3]);

        const double K = params.meas_dist_k;
        double weight  = std::log(K * ypd[2] + 1.0);

        Eigen::Matrix<Scalar, NZ, NZ> Rs = Eigen::Matrix<Scalar, NZ, NZ>::Zero();
        Rs.diagonal() << weight + std::abs(delta_yaw / 3 + 1), weight + std::abs(delta_yaw / 3 + 1),
            weight, log(std::abs(ypd[2]) + 1) / 200 + 9e-2;
        return Rs;
    }
};

struct OutpostModel {
    using Scalar            = double;
    static constexpr int NX = util::O_STATE_MAX;
    static constexpr int NZ = util::MEASURE_MAX; // 量测 [px,py,pz,yaw]

    using VecX = Eigen::Matrix<Scalar, NX, 1>;
    using VecZ = Eigen::Matrix<Scalar, NZ, 1>;

    struct Params {
        // 过程噪声
        Scalar sigma_q_xy  = 10.0;
        Scalar sigma_q_z   = 1.0;
        Scalar sigma_a_yaw = 1.0;

        // 观测噪声（和距离、yaw 误差有关）
        Scalar meas_dist_k = 0.43;  // 距离权重的系数 K
        Scalar yaw_log_k   = 0.005; // yaw 噪声前的系数
    };
    // todo
    int armor_id                           = 0;
    static constexpr double OUTPOST_RADIUS = 0.2765; // 详见机械图纸
    static constexpr double OUTPOST_DZ     = 0.1;    // 两个装甲板之间的铅锤高度差
    static constexpr int ARMORS_NUM        = 3;

    Params params;                                   // 所有可调参数集中在这里

    OutpostModel() = default;
    explicit OutpostModel(const Params& p)
        : params(p) {}

    VecX f(const VecX& x, Scalar dt) const {
        VecX xn;
        using namespace util;
        // clang-format off
        xn << x[O_XC],
              x[O_YC],
              x[O_YAW] + x[O_VYAW]*dt,
              x[O_VYAW],
              x[O_Z0],
              x[O_Z1],
              x[O_Z2];
        // clang-format on
        return xn;
    }

    VecZ h(const VecX& x) const { return h(x, armor_id); }

    VecZ h(const VecX& x, int id) const {
        VecZ z;
        using namespace util;
        assert(id >= 0 && id <= ARMORS_NUM - 1);
        const Scalar angle_step = 2.0 * M_PI / static_cast<Scalar>(ARMORS_NUM);

        double tmp_yaw = x(O_YAW) + angle_step * id;

        z[3] = tmp_yaw;

        z[0] = x(O_XC) - OUTPOST_RADIUS * std::cos(tmp_yaw);
        z[1] = x(O_YC) - OUTPOST_RADIUS * std::sin(tmp_yaw);
        z[2] = x(O_Z0 + id);

        return z;
    }

    Eigen::Matrix<Scalar, NX, NX> Q_sqrt(Scalar dt) const {
        using MatX = Eigen::Matrix<Scalar, NX, NX>;
        MatX Q     = MatX::Zero();
        if (dt <= Scalar(0)) {
            return Q;
        }

        using namespace util;

        const Scalar sigma_xy2  = params.sigma_q_xy * params.sigma_q_xy;
        const Scalar sigma_z2   = params.sigma_q_z * params.sigma_q_z;
        const Scalar sigma_yaw2 = params.sigma_a_yaw * params.sigma_a_yaw;

        const Scalar dt2 = dt * dt;
        const Scalar dt3 = dt2 * dt;
        const Scalar dt4 = dt2 * dt2;

        // 1）X、Y：只有位置，没有速度 “加性随机游走”
        //    噪声大小与 dt^2 成正比即可
        Q(O_XC, O_XC) = sigma_xy2 * dt2;
        Q(O_YC, O_YC) = sigma_xy2 * dt2;

        // 2）YAW / VYAW：是 位置+速度 对 CV模型
        //    [ yaw, vyaw ] 的连续白噪声加速度模型
        Q(O_YAW, O_YAW)   = sigma_yaw2 * dt4 / 4.0;
        Q(O_YAW, O_VYAW)  = sigma_yaw2 * dt3 / 2.0;
        Q(O_VYAW, O_YAW)  = Q(O_YAW, O_VYAW);
        Q(O_VYAW, O_VYAW) = sigma_yaw2 * dt2;

        // 3）Z0, Z1, Z2：各自随机游走（整体高度可能缓慢漂移）
        Q(O_Z0, O_Z0) = sigma_z2 * dt2;
        Q(O_Z1, O_Z1) = sigma_z2 * dt2;
        Q(O_Z2, O_Z2) = sigma_z2 * dt2;

        // 做 Cholesky 分解，返回 S，使得 S * S^T = Q
        Eigen::LLT<MatX> llt(Q);
        MatX S = llt.matrixL();
        return S;
    }

    Eigen::Matrix<Scalar, NZ, Eigen::Dynamic> R_sqrt(const VecZ& z) const {
        auto ypd         = util::xyz2ypd({z[0], z[1], z[2]});
        double delta_yaw = util::shortest_deg(ypd[0], z[3]);

        const double K = params.meas_dist_k;
        double weight  = std::log(K * ypd[2] + 1.0);

        Eigen::Matrix<Scalar, NZ, NZ> Rs = Eigen::Matrix<Scalar, NZ, NZ>::Zero();
        Rs.diagonal() << weight + std::abs(delta_yaw / 3 + 1), weight + std::abs(delta_yaw / 3 + 1),
            weight, log(std::abs(ypd[2]) + 1) / 600 + 9e-2;
        return Rs;
    }
};
