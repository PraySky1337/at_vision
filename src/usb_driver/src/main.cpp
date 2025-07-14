#include "usb/usb_node.hpp"
#include <memory>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    ATLOG_INFO("rclcpp init");
    rclcpp::executors::MultiThreadedExecutor executors;
    auto usb_driver = std::make_shared<usb_driver::UsbDriverNode>();
    executors.add_node(usb_driver);
    executors.spin();
    executors.cancel();
    rclcpp::shutdown();
    return 0;
}