#include "armor_solver/util.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace util {

std::vector<Eigen::Vector4d> getRoboArmorPose(
    const Eigen::Vector3d& target_center, const double yaw, const double radius, const double l,
    const double z0, const double h, const size_t armors_num) {
    (void)z0;
    auto armor_positions = std::vector<Eigen::Vector4d>(armors_num, Eigen::Vector4d::Zero());
    // Calculate the position of each armor
    for (size_t i = 0; i < armors_num; ++i) {
        bool use_l_h = (i == 1 || i == 3);
        double r;
        double z;
        double temp_yaw = yaw + (int)i * M_PI / 2;
        if (use_l_h) {
            r = radius + l;
            z = target_center.z() + h;
        } else {
            r = radius;
            z = target_center.z();
        }
        armor_positions[i] = Eigen::Vector4d(
            target_center.x() - r * std::cos(temp_yaw), target_center.y() - r * std::sin(temp_yaw),
            z, temp_yaw);
    }

    return armor_positions;
}

std::vector<Eigen::Vector4d> getRoboArmorPose(const rm_interfaces::msg::Target& target) {
    const auto& t = target;
    Eigen::Vector3d target_center{t.position.x, t.position.y, t.position.z};

    return getRoboArmorPose(target_center, t.yaw, t.radius0, t.l, t.z0, t.h, t.armors_num);
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
    target.position.x = state(XC);
    target.position.y = state(YC);
    target.position.z = state(ZC);
    target.velocity.x = state(VX);
    target.velocity.y = state(VY);
    target.velocity.z = state(VZ);
    target.z0         = state(ZC);
    target.yaw        = state(YAW);
    target.v_yaw      = state(V_YAW);
    target.radius0    = state(R);
    target.l          = state(L);
    target.h          = state(H);
    return target;
}

Eigen::Vector3d state2armor_xyz(double const* x, int id, int armors_num) {
    auto angle   = limit_rad(x[YAW] + id * 2 * M_PI / armors_num);
    auto use_l_h = (armors_num == 4) && (id == 1 || id == 3);
    auto r       = x[R];
    if (use_l_h)
        r += x[L]; // L = r2 - r1 so...
    auto armor_x = x[XC] - r * std::cos(angle);
    auto armor_y = x[YC] - r * std::sin(angle);
    auto armor_z = use_l_h ? x[ZC] + x[H] : x[ZC];
    return {armor_x, armor_y, armor_z};
}

double limit_rad(double angle) {
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle <= -M_PI)
        angle += 2 * M_PI;
    return angle;
}
} // namespace util
