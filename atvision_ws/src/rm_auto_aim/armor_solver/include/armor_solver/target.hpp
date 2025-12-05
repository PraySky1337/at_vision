#pragma once
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "single_armor_srukf.hpp"
#include "srukf.hpp"
#include <optional>
#include <rclcpp/time.hpp>

namespace armor_tracker {

struct Tracker {
    enum State : uint8_t {
        IDLE,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } state;
    explicit Tracker()
        : state(IDLE)
        , detecting_count(0)
        , lost_count(0) {}
    std::string first_meet_u(const rm_interfaces::msg::Armors& armor);
    void step(double dts, const rm_interfaces::msg::Armors& armors);
    void predict(double dts);
    bool update(const rm_interfaces::msg::Armors& armors);
    rm_interfaces::msg::Target get_target();

    std::string name;
    int armor_num;
    using ImplUKF = srukf::SRUKF<CV3D, CV3D::NX, CV3D::NZ>;
    std::optional<ImplUKF> ukf;

private:
    std::vector<rm_interfaces::msg::Armor> match_all(
        const rm_interfaces::msg::Armors& armors, std::vector<int>& idx,
        const ImplUKF::VecX& x_pre);
    CV3D cv3d_model_;
    void state_machine(bool found);
    int detecting_count;
    int lost_count;
};

} // namespace armor_tracker
