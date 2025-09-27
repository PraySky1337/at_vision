#ifndef HIK_CAMERA_NODE_HPP_
#define HIK_CAMERA_NODE_HPP_

#include "MvCameraControl.h"

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace hik_camera {

class HikCameraNode : public rclcpp::Node {
public:
    explicit HikCameraNode(const rclcpp::NodeOptions& options);
    ~HikCameraNode() override;

private:
    void declareParameters();
    rcl_interfaces::msg::SetParametersResult
    parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    // camera control helpers (重连相关)
    bool openCamera();               // 尝试打开相机（建立 handle、OpenDevice、GetImageInfo、StartGrabbing）
    void closeCamera();              // 优雅关闭相机（StopGrabbing/Close/Destroy）
    bool reconnectLoop();            // 循环重连，直到成功或 exit_flag_

    // ROS msgs / pubs
    sensor_msgs::msg::Image image_msg_;
    image_transport::CameraPublisher camera_pub_;

    // Hikvision SDK
    int nRet = MV_OK;
    void* camera_handle_{nullptr};
    MV_IMAGE_BASIC_INFO img_info_;
    MV_CC_PIXEL_CONVERT_PARAM convert_param_;

    // Camera info manager
    std::string camera_name_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    // Threading / state
    std::atomic_bool exit_flag_{false};
    int fail_count = 0;
    std::thread capture_thread_;

    // parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    // reconnect params
    int reconnect_interval_ms_ = 2000;
    int reconnect_max_attempts_ = -1; // <=0 表示无限次

    // mutex to protect camera_handle_ and SDK calls
    std::mutex camera_mutex_;
};

} // namespace hik_camera

#endif // HIK_CAMERA_NODE_HPP_
