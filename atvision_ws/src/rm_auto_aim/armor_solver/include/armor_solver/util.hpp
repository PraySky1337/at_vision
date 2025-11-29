#pragma once
#include <Eigen/Dense>
#include <rm_interfaces/msg/armors.hpp>
#include <rm_interfaces/msg/target.hpp>

namespace util {
inline std::vector<Eigen::Vector3d> getRoboArmorPositions(
    const Eigen::Vector3d& target_center, const double target_yaw, const double r1, const double r2,
    const double d_zc, const double d_za, const size_t armors_num) noexcept {
    auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
    // Calculate the position of each armor
    bool is_current_pair = true;
    double r = 0., target_dz = 0.;
    for (size_t i = 0; i < armors_num; i++) {
        double temp_yaw =
            target_yaw + static_cast<int>(i) * (2 * M_PI / static_cast<int>(armors_num));
        if (armors_num == 4) {
            r               = is_current_pair ? r1 : r2;
            target_dz       = d_zc + (is_current_pair ? 0 : d_za);
            is_current_pair = !is_current_pair;
        }
        armor_positions[i] =
            target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
    }
    return armor_positions;
}

inline std::vector<Eigen::Vector3d>
    getRoboArmorPositions(const rm_interfaces::msg::Target& target) {
    auto& t = target;
    Eigen::Vector3d target_center{t.position.x, t.position.y, t.position.z};
    return getRoboArmorPositions(
        target_center, t.yaw, t.radius_1, t.radius_2, t.d_zc, t.d_za, t.armors_num);
}

inline std::vector<double> entropyWeight(const std::vector<std::vector<double>>& X) {
    int m = X.size();    // 样本数（armors 个数）
    int n = X[0].size(); // 指标数（4 个）

    std::vector<std::vector<double>> P(m, std::vector<double>(n));
    std::vector<double> e(n, 0.0), w(n, 0.0);

    // 归一化 max-min
    for (int j = 0; j < n; ++j) {
        double xmax = -1e9, xmin = 1e9;
        for (int i = 0; i < m; ++i) {
            xmax = std::max(xmax, X[i][j]);
            xmin = std::min(xmin, X[i][j]);
        }
        for (int i = 0; i < m; ++i) {
            if (xmax == xmin)
                P[i][j] = 0.0;
            else
                P[i][j] = (X[i][j] - xmin) / (xmax - xmin);
        }
    }

    // 熵值
    const double k = 1.0 / std::log(m);
    for (int j = 0; j < n; ++j) {
        double sum = 0.0;
        for (int i = 0; i < m; ++i) {
            if (P[i][j] > 1e-12)
                sum += P[i][j] * std::log(P[i][j]);
        }
        e[j] = -k * sum;
    }

    // 权重
    double denom = 0.0;
    for (double ei : e)
        denom += (1.0 - ei);
    for (int j = 0; j < n; ++j)
        w[j] = (1.0 - e[j]) / denom;

    return w;
}
} // namespace util