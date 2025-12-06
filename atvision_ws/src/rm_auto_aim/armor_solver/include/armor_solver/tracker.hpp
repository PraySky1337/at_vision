#pragma once
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "motion_model.hpp"
#include "srukf.hpp"
#include <optional>
#include <rclcpp/time.hpp>

namespace armor_tracker {

struct Tracker {
    struct Params : public RobotModel::Params {
        int lost_thres;
        int tracking_thres;
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
        params = p;
        motion_model_.params = static_cast<const decltype(motion_model_.params)&>(p);
    }

    std::string name;
    int armor_num;
    using ImplUKF = srukf::SRUKF<RobotModel, RobotModel::NX, RobotModel::NZ>;
    std::optional<ImplUKF> ukf;

    rm_interfaces::msg::Target get_target();
    const auto& get_measurement() const noexcept { return measurement_; }

    std::string first_meet_u(const rm_interfaces::msg::Armors& armor);
    void step(double dts, const rm_interfaces::msg::Armors& armors);
    void predict(double dts);
    bool update(const rm_interfaces::msg::Armors& armors);

private:
    std::vector<rm_interfaces::msg::Armor> match_all(
        const rm_interfaces::msg::Armors& armors, std::vector<int>& idx,
        const ImplUKF::VecX& x_pre);
    void state_machine(bool found);
    void set_measurement(const ImplUKF::VecZ& z, bool another_pair);
    std::array<double, RobotModel::NZ * 2> measurement_;
    RobotModel motion_model_;
    int detecting_count_;
    int lost_count_;
};

} // namespace armor_tracker
