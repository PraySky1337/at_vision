#pragma once
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>

namespace solver {

class Ballistics {
public:
    explicit Ballistics(
        rclcpp::Node* node, const std::string& base_frame = "odom",
        const std::string& muzzle_frame = "muzzle_link")
        : node_(node)
        , base_frame_(base_frame)
        , muzzle_frame_(muzzle_frame) {
        // 参数声明
        v0_ = node_->declare_parameter("ballistics.bullet_velocity", 21.0);
        k_  = node_->declare_parameter("ballistics.resistance_coefficient", 0.0);
        g_  = node_->declare_parameter("ballistics.gravity", 9.81);
        dt_ = node_->declare_parameter("ballistics.integration_step", 1e-3);

        // tf buffer / listener 只需初始化一次
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    /// 给定目标点（base_frame 系），返回 pitch/yaw
    std::pair<double, double> solve(const Eigen::Vector3d& target_in_base) const {
        // 直接查 transform
        Eigen::Isometry3d T_base_muzzle;
        try {
            auto tf_msg = tf_buffer_->lookupTransform(
                base_frame_, muzzle_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
            T_base_muzzle = tf2::transformToEigen(tf_msg.transform);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
            return {0.0, 0.0};
        }

        Eigen::Vector3d muzzle = T_base_muzzle.translation();
        Eigen::Vector3d rel    = target_in_base - muzzle;

        double d = std::hypot(rel.x(), rel.y());
        double h = rel.z();

        double theta = solvePitch(d, h);
        double pitch = -theta;
        double yaw   = std::atan2(rel.y(), rel.x());
        return {pitch, yaw};
    }

    /// 重载：支持 geometry_msgs::msg::Point
    std::pair<double, double> solve(const geometry_msgs::msg::Point& target_in_base) const {
        Eigen::Vector3d target(target_in_base.x, target_in_base.y, target_in_base.z);
        return solve(target);
    }

    struct BallisticsSolution {
        double pitch, yaw, tof;
        bool ok;
    };

    double integrateTimeOfFlight(double theta, double d) const {
        double x = 0.0, y = 0.0, t = 0.0;
        double vx         = v0_ * std::cos(theta);
        double vy         = v0_ * std::sin(theta);
        const double dt   = dt_;
        const double TMAX = 5.0; // 安全上限

        while (x < d && t < TMAX) {
            const double v  = std::hypot(vx, vy);
            const double ax = -k_ * v * vx;
            const double ay = -g_ - k_ * v * vy;

            const double nx = vx * dt;
            // 若本步将跨越 d，用线性比例内插时间
            if (nx > 0.0 && x + nx >= d) {
                const double frac = (d - x) / nx; // 0..1
                t += frac * dt;
                break;
            }
            x += nx;
            y += vy * dt;
            vx += ax * dt;
            vy += ay * dt;
            t += dt;
        }
        return t;
    }

    BallisticsSolution solveWithTof(const geometry_msgs::msg::Point& target) const {
        Eigen::Isometry3d T_base_muzzle;
        try {
            auto tf_msg = tf_buffer_->lookupTransform(
                base_frame_, muzzle_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
            T_base_muzzle = tf2::transformToEigen(tf_msg.transform);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
            return {0, 0, 0, false};
        }

        const Eigen::Vector3d muzzle = T_base_muzzle.translation();
        const Eigen::Vector3d rel(
            target.x - muzzle.x(), target.y - muzzle.y(), target.z - muzzle.z());

        const double d = std::hypot(rel.x(), rel.y());
        const double h = rel.z();
        if (d < 1e-6 && std::abs(h) < 1e-6) {
            RCLCPP_WARN(node_->get_logger(), "Target coincides with muzzle.");
            return {0, 0, 0, false};
        }

        // 俯仰解算（枪口→目标几何），仍然得到的是 odom 系下的目标俯仰
        const double theta = solvePitch(d, h);
        if (!std::isfinite(theta))
            return {0, 0, 0, false};
        const double y_res = simulateY(theta, d) - h;
        if (!std::isfinite(y_res) || std::abs(y_res) > 1e-3)
            return {0, 0, 0, false};

        const double c = std::cos(theta);
        if (std::abs(c) < 1e-6)
            return {0, 0, 0, false};

        double tof = 0.0;
        if (k_ == 0.0) {
            tof = d / (v0_ * c);
        } else {
            tof = integrateTimeOfFlight(theta, d);
            if (!std::isfinite(tof) || tof <= 0.0)
                return {0, 0, 0, false};
        }

        // 直接就是 odom 系绝对角
        const double yaw_odom   = std::atan2(rel.y(), rel.x());
        const double pitch_odom = -theta; // 你的“向下为正”约定

        return {pitch_odom, yaw_odom, tof, true};
    }

private:
    rclcpp::Node* node_;
    std::string base_frame_, muzzle_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double v0_, k_, g_, dt_;

    double simulateY(double theta, double d) const {
        if (k_ == 0.0) {
            double vx = v0_ * std::cos(theta);
            double vy = v0_ * std::sin(theta);
            double t  = d / vx;
            return vy * t - 0.5 * g_ * t * t;
        }
        double x = 0, y = 0;
        double vx = v0_ * std::cos(theta);
        double vy = v0_ * std::sin(theta);
        while (x < d) {
            double v  = std::hypot(vx, vy);
            double ax = -k_ * v * vx;
            double ay = -g_ - k_ * v * vy;
            x += vx * dt_;
            y += vy * dt_;
            vx += ax * dt_;
            vy += ay * dt_;
        }
        return y;
    }

    double solvePitch(double d, double h, double eps = 1e-4, int max_iter = 14) const {
        double lo = 0, hi = M_PI / 3;
        double y_lo = simulateY(lo, d) - h;
        double y_hi = simulateY(hi, d) - h;
        if (y_lo * y_hi > 0)
            return 0.0;

        for (int i = 0; i < max_iter; i++) {
            double mid   = 0.5 * (lo + hi);
            double y_mid = simulateY(mid, d) - h;
            if (std::abs(y_mid) < eps)
                return mid;
            if (y_mid * y_lo < 0)
                hi = mid;
            else {
                lo   = mid;
                y_lo = y_mid;
            }
        }
        return 0.5 * (lo + hi);
    }
};

} // namespace solver
