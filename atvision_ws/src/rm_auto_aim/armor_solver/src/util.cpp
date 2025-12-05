#include "armor_solver/util.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace util {

std::vector<Eigen::Vector4d> get_robo_armor_poses(
    const Eigen::Vector3d& target_pos, const double yaw, const double radius0, const double radius1,
    const double z1, const size_t armors_num) {
    std::vector<Eigen::Vector4d> poses(armors_num);
    const double angle_step = 2.0 * M_PI / static_cast<double>(armors_num);
    const double cx         = target_pos.x();
    const double cy         = target_pos.y();
    const double z0         = target_pos.z();

    for (size_t i = 0; i < armors_num; ++i) {
        const bool another_pair = (armors_num == 4) && (i == 1 || i == 3);
        const double r          = another_pair ? radius1 : radius0;
        const double angle      = normalize_rad(yaw + angle_step * static_cast<double>(i));
        poses[i].x()            = cx - r * std::cos(angle);
        poses[i].y()            = cy - r * std::sin(angle);
        poses[i].z()            = another_pair ? z1 : z0;
        poses[i].w()            = angle;
    }

    return poses;
}

std::vector<Eigen::Vector4d> get_robo_armor_poses(const rm_interfaces::msg::Target& target) {
    Eigen::Vector3d target_center(target.position.x, target.position.y, target.position.z);
    return get_robo_armor_poses(
        target_center, target.yaw, target.radius0, target.radius1, target.z1, target.armors_num);
}

std::vector<double> entropy_weight(const std::vector<std::vector<double>>& X) {
    const int m = static_cast<int>(X.size());    // 样本数
    const int n = static_cast<int>(X[0].size()); // 指标数

    std::vector<std::vector<double>> P(m, std::vector<double>(n));
    std::vector<double> e(n, 0.0), w(n, 0.0);

    // 归一化 max-min
    for (int j = 0; j < n; ++j) {
        double xmax = -1e9;
        double xmin = 1e9;
        for (int i = 0; i < m; ++i) {
            xmax = std::max(xmax, X[i][j]);
            xmin = std::min(xmin, X[i][j]);
        }
        for (int i = 0; i < m; ++i) {
            if (xmax == xmin) {
                P[i][j] = 0.0;
            } else {
                P[i][j] = (X[i][j] - xmin) / (xmax - xmin);
            }
        }
    }

    // 熵值
    const double k = 1.0 / std::log(static_cast<double>(m));
    for (int j = 0; j < n; ++j) {
        double sum = 0.0;
        for (int i = 0; i < m; ++i) {
            if (P[i][j] > 1e-12) {
                sum += P[i][j] * std::log(P[i][j]);
            }
        }
        e[j] = -k * sum;
    }

    // 权重
    double denom = 0.0;
    for (double ei : e) {
        denom += (1.0 - ei);
    }
    for (int j = 0; j < n; ++j) {
        w[j] = (1.0 - e[j]) / denom;
    }

    return w;
}

rm_interfaces::msg::Target state2target(const Eigen::MatrixXd& state) {
    rm_interfaces::msg::Target target;
    // target.position.x = state(XC);
    // target.position.y = state(YC);
    // target.position.z = state(ZC);
    // target.velocity.x = state(VX);
    // target.velocity.y = state(VY);
    // target.velocity.z = state(VZ);
    // target.z0         = state(ZC);
    // target.yaw        = state(YAW);
    // target.v_yaw      = state(V_YAW);
    // target.radius0    = state(R);
    // target.l          = state(L);
    // target.h          = state(H);
    return target;
}

Eigen::Vector3d state2armor_xyz(double const* x, int id, int armors_num) {
    auto angle = normalize_rad(x[YAW] + id * 2 * M_PI / armors_num);
    // auto angle   = normalize_rad(x[YAW] + id * 2 * M_PI / armors_num);
    bool another_pair = (armors_num == 4) && (id == 1 || id == 3);
    double r          = another_pair ? x[R_1] : x[R_0];
    auto armor_x      = x[XC] - r * std::cos(angle);
    auto armor_y      = x[YC] - r * std::sin(angle);
    auto armor_z      = another_pair ? x[Z0] : x[Z0] + x[H];
    return {armor_x, armor_y, armor_z};
}

} // namespace util
