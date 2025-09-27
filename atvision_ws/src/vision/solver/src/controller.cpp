#include "solver/controller.hpp"
#include "solver/planner.hpp"
#include <rcl/time.h>
#include <rclcpp/qos.hpp>

namespace solver {

/* ================= 构造 / 析构 ================= */

Controller::Controller(const rclcpp::NodeOptions& options)
    : rclcpp::Node("Controller", options)
    , ballistics_(this, "odom", "muzzle_link")
    , planner_(this)
    , new_measure_(false) {
    // 订阅 tracker/target（主回调线程仅写入双缓冲）
    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "tracker/target", rclcpp::SensorDataQoS(),
        std::bind(&Controller::targetCallback, this, std::placeholders::_1));
    plan_control_cmd_pub_ = this->create_publisher<auto_aim_interfaces::msg::PlannedControlCmd>(
        "planner/control_command", rclcpp::SensorDataQoS());
    marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("planner/aim_marker", 10);

    // 频率与前推参数
    max_hz_                 = this->declare_parameter("controller.max_hz", 500.0);
    predict_offset_ms_      = this->declare_parameter("controller.predict_offset_ms", 0.0);
    longest_reasoning_time_ = this->declare_parameter("controller.longest_reasoning_time", 1.0);

    rate_controller_.setFrequency(max_hz_.load());

    RCLCPP_INFO(get_logger(), "ROS CLOCK TIME TYPE: %d", this->get_clock()->get_clock_type());
    last_control_stamp_ = this->now();
    running_.store(true, std::memory_order_release);
    worker_thread_ = std::thread(&Controller::workerLoop, this);
}

Controller::~Controller() {
    running_.store(false);
    if (worker_thread_.joinable())
        worker_thread_.join();
}

void Controller::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg) {
    if (msg->tracking) {
        auto copy = *msg;
        target_buf_.write([&](Target& t) { t.fromMsg(copy); });
        new_measure_.store(true, std::memory_order_release);
    }
}

void Controller::workerLoop() {
    Target cur_target;
    bool has_target = false; // ← 是否有可用目标
    rclcpp::Time end_time{0, 0, now().get_clock_type()};

    while (running_.load(std::memory_order_acquire)) {
        // 收到新帧，刷新目标和有效期
        if (new_measure_.exchange(false)) {
            target_buf_.read_into(cur_target);
            end_time   = now() + rclcpp::Duration::from_seconds(longest_reasoning_time_);
            has_target = true;
        }

        // 没有目标 → idle
        if (!has_target) {
            rate_controller_.tick();
            continue;
        }

        // 超过有效期 → 作废
        if ((end_time - now()).seconds() < 0) {
            has_target = false; // ← 关键：目标过期后失效
            rate_controller_.tick();
            RCLCPP_INFO(get_logger(), "wait for next measure");
            continue;
        }

        // 有目标 & 未过期 → 计算
        auto time_now   = now();
        const double dt = (time_now - last_control_stamp_).seconds();

        double infer_time_s = 0.0;
        if (cur_target.stamp.get_clock_type() == now().get_clock_type()) {
            infer_time_s = (time_now - cur_target.stamp).seconds();
        }

        const double pipeline_delay_s = predict_offset_ms_.load() * 1e-3 + dt + infer_time_s;

        FireSolution fire = solveWithIter(cur_target, pipeline_delay_s);
        if (fire.ok) {
            last_tof_           = fire.timeoflight;
            PlanControlCmd plan = planner_.process(fire);
            plan_control_cmd_pub_->publish(plan.toMsg());
            plan_cmd_buf_.write_move(std::move(plan));
        } else {
            RCLCPP_ERROR(get_logger(), "No valid fire solution!");
        }

        last_control_stamp_ = time_now;
        rate_controller_.tick();
    }
}

/* ================= 预测目标状态 + 解算装甲板 ================= */

Target Controller::predictTargetState(const Target& t, double predict_s) {
    Target s;
    s.position.x = t.position.x + t.velocity.x * predict_s;
    s.position.y = t.position.y + t.velocity.y * predict_s;
    s.position.z = t.position.z + t.velocity.z * predict_s;

    s.velocity = t.velocity; // 若需非匀速，可在此插值/外推
    s.yaw      = t.yaw + t.v_yaw * predict_s;
    s.v_yaw    = t.v_yaw;

    s.armors_num = t.armors_num;
    s.radius_1   = t.radius_1;
    s.radius_2   = t.radius_2;
    s.dz         = t.dz;
    return s;
}

std::vector<geometry_msgs::msg::Point> Controller::armorPositionsFromState(const Target& s) {
    const int n = s.armors_num;
    std::vector<geometry_msgs::msg::Point> pts;
    pts.reserve(std::max(0, n));

    bool use_r1 = true;
    for (int i = 0; i < n; ++i) {
        const double a = s.yaw + i * (2.0 * M_PI / n);
        const double r = (n == 4) ? (use_r1 ? s.radius_1 : s.radius_2) : s.radius_1;
        geometry_msgs::msg::Point p;
        p.x = s.position.x - r * std::cos(a);
        p.y = s.position.y - r * std::sin(a);
        p.z = (n == 4) ? (s.position.z + (use_r1 ? 0.0 : s.dz)) : s.position.z;
        pts.emplace_back(std::move(p));
        use_r1 = !use_r1;
    }
    return pts;
}

/* ================= 选最近装甲板 ================= */

geometry_msgs::msg::Point
    Controller::chooseArmor(const std::vector<geometry_msgs::msg::Point>& armors) {
    double best = std::numeric_limits<double>::infinity();
    int idx     = -1;
    for (int i = 0; i < static_cast<int>(armors.size()); ++i) {
        const auto& p = armors[i];
        const double d =
            std::hypot(p.x, p.y); // 与原点（枪口在 odom 原点对齐后靠 Ballistics 平移）投影距离
        if (d < best) {
            best = d;
            idx  = i;
        }
    }
    if (idx < 0) {
        RCLCPP_ERROR(this->get_logger(), "No armor to choose!");
        return geometry_msgs::msg::Point{};
    }
    return armors[idx];
}

/* ================= 弹道 + 自洽时间迭代 ================= */
FireSolution Controller::solveWithIter(const Target& target, double pipeline_delay_s) {
    FireSolution ret{0, 0, 0, false};
    double t               = pipeline_delay_s + std::max(0.0, last_tof_);
    constexpr double eps   = 1e-3;
    constexpr double alpha = 0.618;
    constexpr int kmax     = 5;

    geometry_msgs::msg::Point chosen_armor;  // 最终选择的装甲板中心

    for (int k = 0; k < kmax; ++k) {
        // 1) 状态预测 + 装甲板候选
        Target s    = predictTargetState(target, t);
        auto armors = armorPositionsFromState(s);

        auto armor = chooseArmor(armors);
        chosen_armor = armor;  // 保存下来，循环结束后用于发布

        // 2) 弹道解算
        auto sol = ballistics_.solveWithTof(armor);
        if (!sol.ok)
            return ret;

        // 3) 时间迭代
        const double t_new = pipeline_delay_s + sol.tof;
        if (std::abs(t_new - t) < eps) {
            ret = {sol.pitch, sol.yaw, sol.tof, true};
            break;  // 已收敛，退出循环
        }
        t   = alpha * t_new + (1.0 - alpha) * t;
        ret = {sol.pitch, sol.yaw, sol.tof, true};
    }

    // ---------- 发布 Marker ----------
    if (marker_pub_ && ret.ok) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp    = this->now();
        m.ns   = "chosen_armor";
        m.id   = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;

        m.pose.position = chosen_armor;
        m.pose.orientation.w = 1.0;

        // 球体大小和颜色
        m.scale.x = m.scale.y = m.scale.z = 0.15;  // 15cm 球
        m.color.r = 1.0f;
        m.color.g = 1.0f;
        m.color.b = 1.0f;
        m.color.a = 0.5f;

        // 生命周期 1s
        m.lifetime = rclcpp::Duration::from_seconds(1.0);

        marker_pub_->publish(m);
    }

    return ret;
}


} // namespace solver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(solver::Controller)
