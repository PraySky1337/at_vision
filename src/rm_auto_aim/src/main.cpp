#include "armor_detector/detector_node.hpp"
#include "armor_tracker/tracker_node.hpp"
#include "logger/logger.hpp"
#include "trajectory/trajectory_node.hpp"
#include "usb/usb_node.hpp"

#include <armor_detector/armor.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    ATLOG_INFO("rclcpp init");
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor executors;
    auto detector = std::make_shared<rm_auto_aim::ArmorDetectorNode>("armor_detector", "", options);
    auto tracker  = std::make_shared<rm_auto_aim::ArmorTrackerNode>(
        detector.get(), "armor_tracker", "", options);
    auto usb_driver = std::make_shared<usb_driver::UsbDriverNode>("usb_driver", "", options);
    auto trajectory = std::make_shared<trajectory::TrajectoryNode>("trajectory", "", options);
    executors.add_node(detector);
    executors.add_node(tracker);
    executors.add_node(usb_driver);
    executors.add_node(trajectory);
    executors.spin();
    executors.cancel();
    rclcpp::shutdown();
    return 0;
}