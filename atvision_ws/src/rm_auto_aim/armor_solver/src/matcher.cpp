#include "armor_solver/matcher.hpp"
#include "armor_solver/util.hpp"
#include <angles/angles.h>
#include <tf2/utils.hpp>

namespace armor_tracker {
void Matcher::filter(rm_interfaces::msg::Armors& armors, const rm_interfaces::msg::Target& target) {
    auto positions = util::getRoboArmorPositions(target);
    if (armors.armors.empty() || positions.empty()) {
        return;
    }

    // 按范数排序，保留最外侧两个特征点
    std::sort(
        positions.begin(), positions.end(),
        [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.norm() < b.norm(); });

    if (positions.size() > 2) {
        positions.erase(positions.begin(), positions.begin() + 2);
    }
    const int M = static_cast<int>(positions.size());     // 特征点数，期望 2
    const int N = static_cast<int>(armors.armors.size()); // 装甲数
    if (M == 0 || N == 0)
        return;

    struct Candidate {
        int pos_idx;                                      // 第几个特征点（0 ~ M-1）
        int armor_idx;                                    // 第几个 armor（0 ~ N-1）
        struct ScoreItem {
            double angle_diff;
            double pos_diff;
            double id_match;                              // 1 完全匹配，0 不匹配，可加权
        } s;                                              // 各维差值
        double score;                                     // 加权后的总分（越小越好）
    };

    std::vector<Candidate> cands;
    std::vector<std::vector<double>> X;                   // 用于熵权法的原始指标矩阵

    // 1) 生成所有 (position, armor) 组合的评分
    for (int j = 0; j < N; ++j) {
        const auto& armor = armors.armors[j];

        tf2::Quaternion tf_q;
        tf2::fromMsg(armor.pose.orientation, tf_q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

        Eigen::Vector3d p(armor.pose.position.x, armor.pose.position.y, armor.pose.position.z);

        for (int i = 0; i < M; ++i) {
            Candidate c;
            c.pos_idx   = i;
            c.armor_idx = j;

            // 角度差
            c.s.angle_diff = std::abs(angles::shortest_angular_distance(yaw, target.yaw));

            // 距离差
            c.s.pos_diff = (p - positions[i]).norm();

            // ID 匹配（反向：1 匹配 -> 指标用 0，越小越好）
            c.s.id_match = (armor.number == target.id) ? 1.0 : 0.0;

            // 熵权法用的原始指标（都“越小越好”）
            X.push_back({c.s.angle_diff, c.s.pos_diff, 1.0 - c.s.id_match});

            cands.push_back(c);
        }
    }

    if (cands.empty())
        return;

    const int K   = static_cast<int>(cands.size()); // 候选数 = M * N
    const int dim = 4;                              // 指标数

    // 2) 计算各指标权重（熵权法）
    auto w = util::entropyWeight(X); // w.size() == 4

    // 3) 按列做一次 0~1 归一化，然后用权重加权求总分
    std::vector<std::vector<double>> P(K, std::vector<double>(dim));
    for (int j = 0; j < dim; ++j) {
        double xmax = -1e9, xmin = 1e9;
        for (int i = 0; i < K; ++i) {
            xmax = std::max(xmax, X[i][j]);
            xmin = std::min(xmin, X[i][j]);
        }
        for (int i = 0; i < K; ++i) {
            if (xmax == xmin)
                P[i][j] = 0.0;
            else
                P[i][j] = (X[i][j] - xmin) / (xmax - xmin);
        }
    }

    for (int i = 0; i < K; ++i) {
        double s = 0.0;
        for (int j = 0; j < dim; ++j) {
            s += w[j] * P[i][j];
        }
        cands[i].score = s; // 越小越好
    }

    // 依据 score 进行二分匹配：选出不冲突的最多 2 对 (pos_idx, armor_idx)
    std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
        return a.score < b.score;
    });

    std::vector<bool> pos_used(M, false);
    std::vector<bool> armor_used(N, false);
    std::vector<int> chosen_armor_indices; // 最后筛出的 armor 下标（<=2）

    for (const auto& c : cands) {
        if (pos_used[c.pos_idx] || armor_used[c.armor_idx]) {
            continue;                      // 该特征点或 armor 已经被占用
        }
        pos_used[c.pos_idx]     = true;
        armor_used[c.armor_idx] = true;
        chosen_armor_indices.push_back(c.armor_idx);
        if (static_cast<int>(chosen_armor_indices.size()) >= 2)
            break;                         // 只要两个匹配
    }

    // 5) 根据匹配结果过滤 armors，只保留选中的那两块（或更少）
    if (!chosen_armor_indices.empty()) {
        rm_interfaces::msg::Armors filtered;
        filtered.header = armors.header;

        for (int idx : chosen_armor_indices) {
            filtered.armors.push_back(armors.armors[idx]);
        }
        armors = std::move(filtered);
    }
}
} // namespace armor_tracker