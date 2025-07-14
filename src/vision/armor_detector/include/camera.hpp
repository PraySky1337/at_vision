#pragma once
#include <cv_bridge/cv_bridge.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <hikcamera/image_capturer.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
namespace camera
{

struct HikCameraParams
{
  int rate;  // hz
  std::string camera_name;
  std::string cam_info_url;
  hikcamera::ImageCapturer::CameraProfile camera_profile;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle;

  explicit HikCameraParams(rclcpp::Node * node)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    rate = node->declare_parameter<int>("rate", 200, desc);

    camera_name = node->declare_parameter("camera.name", "camera", desc);
    cam_info_url = node->declare_parameter(
      "camera.cam_info_url", "package://rm_auto_aim/config/camera_info.yaml", desc);

    int exp_ms = node->declare_parameter<int>("camera.exposure_time", 4, desc);
    camera_profile.exposure_time = std::chrono::duration<float, std::micro>(exp_ms);
    camera_profile.gain = node->declare_parameter("camera.gain", 8.0f, desc);
    camera_profile.invert_image = node->declare_parameter("camera.invert", false, desc);
    camera_profile.trigger_mode = node->declare_parameter("camera.trigger_mode", false, desc);
    camera_profile.gain = std::clamp(camera_profile.gain, 0.0f, 16.0f);
  }
};

}  // namespace camera
