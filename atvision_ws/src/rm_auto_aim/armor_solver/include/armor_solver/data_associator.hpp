#pragma once

#include <Eigen/Dense>
#include <limits>
#include <vector>

#include "rm_interfaces/msg/armor.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "util.hpp"

namespace rm_auto_aim {

// Template-based data associator for armor matching
// Eliminates code duplication between robot and outpost matching
template<typename Model>
class DataAssociator {
public:
    static constexpr int NZ = Model::NZ;
    static constexpr int NX = Model::NX;

    using VecZ = typename Model::VecZ;
    using VecX = Eigen::Matrix<double, NX, 1>;
    using MatXX = Eigen::Matrix<double, NX, NX>;
    using MatZ = Eigen::Matrix<double, NZ, NZ>;

    struct MatchResult {
        std::vector<rm_interfaces::msg::Armor> armors;
        std::vector<int> armor_ids;
    };

    MatchResult match(
        const rm_interfaces::msg::Armors& observations,
        const VecX& x_pred,
        const MatXX& Sx_pred,
        Model& model,
        const std::string& target_name,
        double gate_threshold,
        int armors_num) {

        MatchResult result;

        if (observations.armors.empty()) {
            return result;
        }

        const int n_obs = static_cast<int>(observations.armors.size());

        // Cost matrix: observation j vs armor_id k
        std::vector<std::vector<double>> cost(
            n_obs, std::vector<double>(armors_num, std::numeric_limits<double>::infinity()));

        // Pre-compute measurements for each observation
        std::vector<VecZ> meas_list(n_obs);
        for (int j = 0; j < n_obs; ++j) {
            const auto& a = observations.armors[j];
            const auto& p = a.pose.position;
            const auto ypd = util::xyz2ypd({p.x, p.y, p.z});
            VecZ z;
            z[0] = ypd[0];
            z[1] = ypd[1];
            z[2] = ypd[2];
            z[3] = util::orientation2yaw(a.pose.orientation);
            meas_list[j] = z;
        }

        // Pre-compute predicted measurements and covariances for each armor_id
        struct PredMeas {
            VecZ z_pred;
            MatZ Pzz;
        };
        std::vector<PredMeas> pred_cache(armors_num);

        const double gamma = std::sqrt(static_cast<double>(NX));
        const double w = 1.0 / (2.0 * static_cast<double>(NX));

        for (int id = 0; id < armors_num; ++id) {
            const VecZ z0 = model.h(x_pred, id);

            std::array<VecZ, 2 * NX> z_sig{};
            for (int k = 0; k < NX; ++k) {
                const VecX xp = x_pred + gamma * Sx_pred.col(k);
                const VecX xm = x_pred - gamma * Sx_pred.col(k);

                VecZ zp = model.h(xp, id);
                VecZ zm = model.h(xm, id);

                zp[0] = util::unwrap_rad(z0[0], zp[0]);
                zp[1] = util::unwrap_rad(z0[1], zp[1]);
                zp[3] = util::unwrap_rad(z0[3], zp[3]);

                zm[0] = util::unwrap_rad(z0[0], zm[0]);
                zm[1] = util::unwrap_rad(z0[1], zm[1]);
                zm[3] = util::unwrap_rad(z0[3], zm[3]);

                z_sig[k] = zp;
                z_sig[NX + k] = zm;
            }

            VecZ z_bar = VecZ::Zero();
            for (const auto& zs : z_sig) {
                z_bar.noalias() += w * zs;
            }

            MatZ Pzz = MatZ::Zero();
            for (const auto& zs : z_sig) {
                VecZ dz;
                dz[0] = util::shortest_rad(z_bar[0], zs[0]);
                dz[1] = zs[1] - z_bar[1];
                dz[2] = zs[2] - z_bar[2];
                dz[3] = util::shortest_rad(z_bar[3], zs[3]);
                Pzz.noalias() += w * (dz * dz.transpose());
            }

            pred_cache[id] = {z_bar, Pzz};
        }

        // Compute costs for each (observation, armor_id) pair
        for (int j = 0; j < n_obs; ++j) {
            if (observations.armors[j].number != target_name) {
                continue;  // Only match armors with matching ID
            }

            for (int id = 0; id < armors_num; ++id) {
                const auto& z_pred = pred_cache[id].z_pred;

                VecZ nu;
                nu[0] = util::shortest_rad(z_pred[0], meas_list[j][0]);
                nu[1] = meas_list[j][1] - z_pred[1];
                nu[2] = meas_list[j][2] - z_pred[2];
                nu[3] = util::shortest_rad(z_pred[3], meas_list[j][3]);

                const Eigen::Matrix<double, NZ, 1> R_diag = model.R_diag(meas_list[j]);
                MatZ S = pred_cache[id].Pzz;
                S.diagonal() += R_diag;
                S = (S + S.transpose()) * 0.5;
                S.diagonal().array() += 1e-9;

                const Eigen::LDLT<MatZ> ldlt(S);
                if (ldlt.info() != Eigen::Success) {
                    continue;
                }

                double d2 = nu.transpose() * ldlt.solve(nu);

                // Gating
                if (std::isfinite(d2) && d2 < gate_threshold) {
                    cost[j][id] = d2;
                }
            }
        }

        // Greedy one-to-one assignment
        std::vector<bool> used_obs(n_obs, false);
        std::vector<bool> used_id(armors_num, false);

        while (true) {
            double best = std::numeric_limits<double>::infinity();
            int best_j = -1;
            int best_id = -1;

            for (int j = 0; j < n_obs; ++j) {
                if (used_obs[j]) continue;
                for (int id = 0; id < armors_num; ++id) {
                    if (used_id[id]) continue;
                    if (cost[j][id] < best) {
                        best = cost[j][id];
                        best_j = j;
                        best_id = id;
                    }
                }
            }

            if (best_j < 0 || best_id < 0) {
                break;
            }

            used_obs[best_j] = true;
            used_id[best_id] = true;

            result.armors.push_back(observations.armors[best_j]);
            result.armor_ids.push_back(best_id);
        }

        return result;
    }
};

} // namespace rm_auto_aim

