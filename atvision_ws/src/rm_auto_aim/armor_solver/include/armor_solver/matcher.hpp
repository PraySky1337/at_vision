#pragma once
#include <rclcpp/logging.hpp>
#include <rm_interfaces/msg/armors.hpp>
#include <rm_interfaces/msg/target.hpp>
#include <limits>
#include <vector>

namespace armor_tracker {
struct Matcher {
    explicit Matcher(const rclcpp::Logger& logger, bool verbose = false)
        : logger_(logger)
        , verbose_(verbose) {}
    void setVerbose(bool v) { verbose_ = v; }
    void setConstraints(double max_match_distance, double max_match_yaw_diff, bool enforce_id) {
        max_match_distance_ = max_match_distance;
        max_match_yaw_diff_ = max_match_yaw_diff;
        enforce_id_         = enforce_id;
    }
    std::vector<int> filter(
        rm_interfaces::msg::Armors& armors, const rm_interfaces::msg::Target& target) const;

private:
    const rclcpp::Logger& logger_;
    bool verbose_{false};
    double max_match_distance_{std::numeric_limits<double>::infinity()};
    double max_match_yaw_diff_{std::numeric_limits<double>::infinity()};
    bool enforce_id_{false};
};
} // namespace armor_tracker
