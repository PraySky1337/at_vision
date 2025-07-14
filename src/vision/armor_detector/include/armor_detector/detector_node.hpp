/*
 * Refactored ArmorDetectorNode with optimized QoS for image subs,
 * dynamic parameter callback handling,
 * multi-threaded CallbackGroups for parallel image callbacks,
 * including publishArmorsAndMarkers implementation.
 */

// detector_node.hpp
#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

#include <atomic>
#include <camera_info_manager/camera_info_manager.hpp>
#include <hikcamera/image_capturer.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "armor_detector/detector.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "camera.hpp"
#include "utility.hpp"

namespace rm_auto_aim {

struct MatStamped {
    cv::Mat img;
    std_msgs::msg::Header header;
};

struct DebugPublishers {
    DebugPublishers(rclcpp::Node* node_ptr);
    ~DebugPublishers();
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors;
    image_transport::Publisher binary;
    image_transport::Publisher numbers;
    image_transport::Publisher result;
    image_transport::Publisher img_raw;
};

class ArmorDetectorNode : public rclcpp::Node {
public:
    explicit ArmorDetectorNode(
        const std::string& name = "armor_detector", const std::string& ns = "",
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~ArmorDetectorNode() override;

private:
    void detectLoop();
    void imageCallback(std_msgs::msg::Header::ConstSharedPtr header);
    void detectOnce(const cv::Mat& raw_img, const std_msgs::msg::Header& header);

    void initDetectors();
    void publishArmorsAndMarkers(const std::vector<Armor>& armors);
    rcl_interfaces::msg::SetParametersResult
        onParametersSet(const std::vector<rclcpp::Parameter>& params);

    // Publishers & Subscribers
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr img_header_pub_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr img_header_sub_;

    // Camera
    std::unique_ptr<hikcamera::ImageCapturer> img_capturer_;
    camera::HikCameraParams hik_camera_params_;
    // Detector & PnP
    std::unique_ptr<Detector> detector_;
    std::shared_ptr<PnPSolver> pnp_solver_;
    camera_info_manager::CameraInfoManager cam_info_manager_;

    // thread
    std::unique_ptr<std::thread> detect_thread;
    std::unique_ptr<std::thread> capture_thread;

    // Camera info
    cv::Point2f cam_center_;

    // Debug
    std::atomic<bool> debug_enabled_{false};
    std::shared_ptr<DebugPublishers> debug_pubs_;

    // Parameter client
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

} // namespace rm_auto_aim
#endif // ARMOR_DETECTOR__DETECTOR_NODE_HPP_