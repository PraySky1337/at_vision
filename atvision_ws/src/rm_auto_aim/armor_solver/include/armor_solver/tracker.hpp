#pragma once
#include "motion_model.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "srukf.hpp"
#include <array>
#include <optional>
#include <rclcpp/time.hpp>

namespace armor_tracker {

struct Tracker {
    struct Params {
        int lost_thres;
        int tracking_thres;
        double matcher_gate;
        double outpost_matcher_gate;
        RobotModel::Params robot_params;
        OutpostModel::Params outpost_params;
    } params;
    enum State : uint8_t {
        IDLE,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } state;
    explicit Tracker()
        : state(IDLE)
        , detecting_count_(0)
        , lost_count_(0) {}
    explicit Tracker(const Params& param)
        : Tracker() {
        set_params(param);
    }

    void set_params(const Params& p) {
        params                = p;
        robot_model_.params   = params.robot_params;
        outpost_model_.params = params.outpost_params;
    }

    std::string name;
    int armor_num;
    using RoboUKF    = srukf::SRUKF<RobotModel, RobotModel::NX, RobotModel::NZ>;
    using OutpostUKF = srukf::SRUKF<OutpostModel, OutpostModel::NX, OutpostModel::NZ>;

    std::optional<RoboUKF> robot_ukf;
    std::optional<OutpostUKF> outpost_ukf;

    rm_interfaces::msg::Target get_target();
    const auto& get_measurement() const noexcept { return measurement_; }

    std::string first_meet_u(const rm_interfaces::msg::Armors& armor);
    void step(double dts, const rm_interfaces::msg::Armors& armors);
    void predict(double dts);
    bool update(const rm_interfaces::msg::Armors& armors);

private:
    std::vector<rm_interfaces::msg::Armor> match_all(
        const rm_interfaces::msg::Armors& armors, std::vector<int>& idx,
        const RoboUKF::VecX& x_pre);
    std::vector<rm_interfaces::msg::Armor> match_all_outpost(
        const rm_interfaces::msg::Armors& armors, std::vector<int>& idx,
        const OutpostUKF::VecX& x_pre);
    void state_machine(bool found);
    void set_measurement(const RoboUKF::VecZ& z, bool another_pair);
    std::array<double, RobotModel::NZ * 2> measurement_;

    // motion_model
    RobotModel robot_model_;
    OutpostModel outpost_model_;
    int detecting_count_;
    std::map<int, std::string> voter_;
    int lost_count_;
};

} // namespace armor_tracker
