#pragma once
#include <auto_aim_interfaces/msg/target.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>


#include "ballistics.hpp"
#include "double_buffer.hpp"
#include "planner.hpp" // ← DI 版 Planner
#include "target.hpp"
#include "util/rate.hpp"

#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <thread>

namespace solver {

struct Controller : public rclcpp::Node {
    explicit Controller(const rclcpp::NodeOptions& options);
    ~Controller();

private:
    // ---------- 线程与通信 ----------
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::PlannedControlCmd>::SharedPtr plan_control_cmd_pub_;

    std::thread worker_thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> new_measure_{false};
    std::atomic<double> longest_reasoning_time_{1.0};

    AtomicDoubleBuffer<Target> target_buf_;
    AtomicDoubleBuffer<PlanControlCmd> plan_cmd_buf_; // 高频结果对外暴露（非 ROS）

    // ---------- 频率与延时 ----------
    std::atomic<double> max_hz_;
    std::atomic<double> predict_offset_ms_; // 额外前推量
    util::Rate rate_controller_;
    rclcpp::Time last_control_stamp_;       // 上次控制时间（ROS Time）
    double last_tof_{0.0};                  // 上次弹道飞行时间

    // ---------- 解算器 ----------
    Ballistics ballistics_;
    Planner planner_;

    std::mutex param_mutex_;

    // ---------- 回调 ----------
    void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
    void workerLoop();

    // ---------- 逻辑拆分 ----------
    Target predictTargetState(const Target& t, double predict_s);
    std::vector<geometry_msgs::msg::Point> armorPositionsFromState(const Target& s);

    geometry_msgs::msg::Point chooseArmor(const std::vector<geometry_msgs::msg::Point>& armors);

    FireSolution solveWithIter(const Target& target, double pipeline_delay_s);

    // ---- 可视化 Publisher ----
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

};

} // namespace solver
