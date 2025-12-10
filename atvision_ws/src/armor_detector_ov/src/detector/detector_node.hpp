// armor_detector_ov/detector_node.hpp
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

// ROS2
#include <image_transport/publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// 项目
#include "armor_detector_ov/ov_model_base.hpp"
#include "pnp_solver.hpp"
#include "rm_interfaces/msg/armors.hpp"

namespace rm_auto_aim {

class ArmorDetectorOVNode : public rclcpp::Node {
public:
    explicit ArmorDetectorOVNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmorDetectorOVNode() override;

private:
    // 回调
    void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    rcl_interfaces::msg::SetParametersResult
        onSetParameters(const std::vector<rclcpp::Parameter>& parameters);
    void publishMarkers(const rm_interfaces::msg::Armors& armors_msg) noexcept;
    rm_interfaces::msg::Armors handleDets(std::vector<ArmorObject>& armors);
    void filteredArmors(std::vector<ArmorObject>& armors);

    // 可视化
    static void drawResults(
        cv::Mat& src, const std::vector<ArmorObject>& armor_objects, const cv::Rect& roi) noexcept;
    static std::string color_letter_(int color);

private:
    // 推理（通过管理器创建具体模型）
    std::unique_ptr<OVModelBase> model_;
    std::unique_ptr<PnPSolver> pnp_solver_;
    std::string model_name_;
    std::string model_path_;
    std::string device_name_  = "CPU";
    bool debug_               = true;
    bool use_ba_              = false;
    bool enable_multi_thread_ = false;

    std::string detect_color_ = "RED";

    // TF/坐标
    std::string odom_frame_ = "odom";
    Eigen::Matrix3d imu_to_camera_{Eigen::Matrix3d::Identity()};
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // 参数回调
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        on_set_parameters_callback_handle_;

    // 订阅/发布
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    image_transport::Publisher result_img_pub_;
    rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Marker 复用对象
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;

    // 相机信息（可选）
    cv::Point2f cam_center_{0.f, 0.f};
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

    // Armors（若需发布）
    rm_interfaces::msg::Armors armors_msg_;
};

} // namespace rm_auto_aim
