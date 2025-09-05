#include "usb/usb_node.hpp"

namespace usb_driver {
using usb_driver::UsbDriver;
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(usb_driver::UsbDriver)
