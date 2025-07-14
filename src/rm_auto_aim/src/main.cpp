#include "armor_detector/detector_node.hpp"
#include "armor_tracker/tracker_node.hpp"
#include "logger/logger.hpp"
#include "usb/usb_node.hpp"

#include <armor_detector/armor.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    ATLOG_INFO("rclcpp init");
    rclcpp::executors::MultiThreadedExecutor executors;
    auto detector   = std::make_shared<rm_auto_aim::ArmorDetectorNode>();
    auto tracker    = std::make_shared<rm_auto_aim::ArmorTrackerNode>(detector.get());
    auto usb_driver = std::make_shared<usb_driver::UsbDriverNode>();
    executors.add_node(detector);
    executors.add_node(tracker);
    executors.add_node(usb_driver);
    executors.spin();
    executors.cancel();
    rclcpp::shutdown();
    return 0;
}