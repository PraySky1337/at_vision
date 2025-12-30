// armor_detector_ov/detector_node.hpp
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
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
#include "armor_detector_ov/detector_backend.hpp"
#include "pnp_solver.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"

namespace rm_auto_aim {

class ArmorDetectorOVNode : public rclcpp::Node {
public:
    explicit ArmorDetectorOVNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmorDetectorOVNode() override;

private:
    // 回调
    void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    void targetCallback(rm_interfaces::msg::Target::ConstSharedPtr target_msg);
    rcl_interfaces::msg::SetParametersResult
        onSetParameters(const std::vector<rclcpp::Parameter>& parameters);
    void publishMarkers(const rm_interfaces::msg::Armors& armors_msg) noexcept;
    rm_interfaces::msg::Armors
        handleDets(std::vector<ArmorObject>& armors, const Eigen::Matrix3d* imu_to_camera);
    void filteredArmors(std::vector<ArmorObject>& armors);

    // 发布检测结果
    void publishArmors(std::vector<ArmorObject>& armors,
                       const rclcpp::Time& stamp,
                       const std::string& frame_id);

    // 可视化
    static void drawResults(
        cv::Mat& src, const std::vector<ArmorObject>& armor_objects, const cv::Rect& roi) noexcept;
    static std::string color_letter_(int color);

    // 工厂方法：创建后端
    std::unique_ptr<DetectorBackend> createBackend();

private:
    // 统一后端接口
    std::unique_ptr<DetectorBackend> detector_;

    // PnP求解器
    std::unique_ptr<PnPSolver> pnp_solver_;

    // 后端类型选择: "nn" 或 "traditional"
    std::string backend_type_ = "nn";

    // NN后端参数
    std::string model_name_;
    std::string model_path_;
    std::string device_name_  = "AUTO";
    std::string device_priorities_ = "GPU,CPU";
    bool enable_multi_thread_ = false;
    int pipeline_queue_size_  = 4;
    int pipeline_num_requests_ = 4;

    // 通用参数
    bool debug_               = true;
    bool use_ba_              = true;
    bool draw_latency_        = true;
    std::string detect_color_ = "RED";

    // 回调组
    rclcpp::CallbackGroup::SharedPtr img_callback_group_;
    rclcpp::CallbackGroup::SharedPtr other_callback_group_;

    // TF/坐标
    std::string odom_frame_ = "odom";
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // 参数回调
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        on_set_parameters_callback_handle_;

    // 订阅/发布
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr target_sub_;
    image_transport::Publisher result_img_pub_;
    rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Marker 复用对象
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;

    // 相机信息（可选）
    std::mutex cam_info_mutex_;
    cv::Point2f cam_center_{0.f, 0.f};
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

    // Armors（若需发布）
    rm_interfaces::msg::Armors armors_msg_;

    // ROI 生成/使用（基于 armor_solver/target）- 仅 NN 后端使用
    bool use_roi_ = true;
    std::string camera_frame_ = "camera_optical_frame";
    double roi_timeout_s_ = 0.2;
    double bbox_timeout_s_ = 0.2;
    double roi_future_tolerance_s_ = 0.02;
    double roi_disable_dist_m_ = 1.5;
    double roi_scale_ = 1.15;
    double roi_smooth_alpha_ = 0.35;
    bool roi_clear_on_miss_ = true;
    double roi_miss_disable_s_ = 0.12;
    bool drop_out_of_order_ = true;
    rclcpp::Time last_sync_pub_stamp_{0, 0, RCL_ROS_TIME};

    std::mutex roi_mutex_;
    std::optional<cv::Rect> next_roi_;
    rclcpp::Time next_roi_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time roi_disable_until_{0, 0, RCL_ROS_TIME};
    std::optional<cv::Rect> last_bbox_;
    rclcpp::Time last_bbox_stamp_{0, 0, RCL_ROS_TIME};
    cv::Size last_image_size_{0, 0};

    // 帧时间戳映射（用于异步模式结果匹配）
    std::mutex stamp_map_mutex_;
    std::unordered_map<uint64_t, std::pair<rclcpp::Time, std::string>> frame_stamp_map_;
};

} // namespace rm_auto_aim
