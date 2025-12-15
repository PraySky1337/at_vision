#pragma once
#include <rclcpp/logging.hpp>

namespace rm_gimbal {

struct Solver {
    explicit Solver(const rclcpp::Logger& logger);

private:
    rclcpp::Logger logger_;

};

}