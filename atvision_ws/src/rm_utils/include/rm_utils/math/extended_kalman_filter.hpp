// Copyright Chen Jun 2023. Licensed under the MIT License.
// Copyright xinyang 2021.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RM_UTILS_KALMAN_FILTER_HPP_
#define RM_UTILS_KALMAN_FILTER_HPP_

// Eigen
#include <Eigen/Dense>
// ceres
#include <ceres/jet.h>

#include "kalman_filter_base.hpp"

namespace at {

template <int N_X, int N_Z, class PredictFunc, class MeasureFunc>
class ExtendedKalmanFilter : public KalmanFilterBase<N_X, N_Z> {
public:
    using Base = KalmanFilterBase<N_X, N_Z>;
    using typename Base::MatrixXX;
    using typename Base::MatrixXZ;
    using typename Base::MatrixZX;
    using typename Base::MatrixZZ;
    using typename Base::MatrixX1;
    using typename Base::MatrixZ1;
    using typename Base::UpdateQFunc;
    using typename Base::UpdateRFunc;

    explicit ExtendedKalmanFilter(
        const PredictFunc& f,
        const MeasureFunc& h,
        const UpdateQFunc& u_q,
        const UpdateRFunc& u_r,
        const MatrixXX& P0) noexcept
        : f_(f), h_(h) {
        this->update_Q_ = u_q;
        this->update_R_ = u_r;
        P_post_ = P0;
        F_.setZero();
        H_.setZero();
    }

    void setState(const MatrixX1& x0) noexcept override { x_post_ = x0; }

    MatrixX1 predict() noexcept override {
        ceres::Jet<double, N_X> x_jet[N_X];
        for (int i = 0; i < N_X; ++i) {
            x_jet[i].a = x_post_[i];
            x_jet[i].v.setZero();
            x_jet[i].v[i] = 1.0;
        }

        ceres::Jet<double, N_X> x_next[N_X];
        f_(x_jet, x_next);

        for (int i = 0; i < N_X; ++i) {
            x_pri_[i] = x_next[i].a;
            F_.block(i, 0, 1, N_X) = x_next[i].v.transpose();
        }

        Q_ = this->update_Q_();
        P_pri_ = F_ * P_post_ * F_.transpose() + Q_;
        x_post_ = x_pri_;
        P_post_ = P_pri_;
        return x_pri_;
    }

    MatrixX1 update(const MatrixZ1& z) noexcept override {
        ceres::Jet<double, N_X> x_jet[N_X];
        for (int i = 0; i < N_X; ++i) {
            x_jet[i].a = x_pri_[i];
            x_jet[i].v.setZero();
            x_jet[i].v[i] = 1.0;
        }

        ceres::Jet<double, N_X> z_jet[N_Z];
        h_(x_jet, z_jet);

        MatrixZ1 z_pri;
        for (int i = 0; i < N_Z; ++i) {
            z_pri[i] = z_jet[i].a;
            H_.block(i, 0, 1, N_X) = z_jet[i].v.transpose();
        }

        R_ = this->update_R_(z);
        K_ = P_pri_ * H_.transpose() * (H_ * P_pri_ * H_.transpose() + R_).inverse();
        x_post_ = x_post_ + K_ * (z - z_pri);
        P_post_ = (MatrixXX::Identity() - K_ * H_) * P_pri_;
        return x_post_;
    }
    void setPredictFunc(const PredictFunc& f) noexcept { f_ = f; }
    void setMeasureFunc(const MeasureFunc& h) noexcept { h_ = h; }

    const MatrixX1& state() const noexcept override { return x_post_; }
    const MatrixXX& covariance() const noexcept override { return P_post_; }

private:
    PredictFunc f_;
    MeasureFunc h_;

    MatrixXX F_{MatrixXX::Zero()};
    MatrixZX H_{MatrixZX::Zero()};
    MatrixXX Q_{MatrixXX::Zero()};
    MatrixZZ R_{MatrixZZ::Zero()};
    MatrixXZ K_{MatrixXZ::Zero()};
    MatrixXX P_pri_{MatrixXX::Identity()};
    MatrixXX P_post_{MatrixXX::Identity()};
    MatrixX1 x_pri_{MatrixX1::Zero()};
    MatrixX1 x_post_{MatrixX1::Zero()};
};


} // namespace fyt

#endif // RM_UTILS_KALMAN_FILTER_HPP_
