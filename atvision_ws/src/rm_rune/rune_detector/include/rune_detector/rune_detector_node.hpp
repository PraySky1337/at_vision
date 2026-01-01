#ifndef RUNE_DETECTOR_RUNE_DETECTOR_NODE_HPP_
#define RUNE_DETECTOR_RUNE_DETECTOR_NODE_HPP_

// std
#include <algorithm>
#include <array>
#include <string>
#include <vector>
// ros2
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// 3rd party
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
// project
#include "rm_interfaces/msg/point2d.hpp"
#include "rm_interfaces/msg/rune_target.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rune_detector/rune_detector.hpp"

namespace fyt::rune {

class RuneDetectorNode : public rclcpp::Node {
public:
    RuneDetectorNode(const rclcpp::NodeOptions& options);
    ~RuneDetectorNode();

private:
    std::unique_ptr<RuneDetector> initDetector();

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    void createDebugPublishers();
    void destroyDebugPublishers();

    void timerCallback();

    // Dynamic Parameter
    rcl_interfaces::msg::SetParametersResult
        onSetParameters(std::vector<rclcpp::Parameter> parameters);
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
    // Image subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

    std::string frame_id_;

    // Rune detector
    std::unique_ptr<RuneDetector> rune_detector_;

    // Rune params
    EnemyColor detect_color_;
    bool is_rune_;

    int arrow_threshold_;
    int target_threshold_;
    int rcenter_threshold_;

    // Debug infomation
    bool debug_;
    bool publish_binary_;
    image_transport::Publisher result_img_pub_;
    image_transport::Publisher arrow_binary_pub_;
    image_transport::Publisher target_binary_pub_;
    image_transport::Publisher rcenter_binary_pub_;

    // //TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 相机内参
    cv::Mat cameraMatrix_;
    // 畸变系数
    cv::Mat distCoeffs_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

    visualization_msgs::msg::Marker
        create_r_marker(const rclcpp::Time& now, const Eigen::Vector3d& r_center_world) {
        visualization_msgs::msg::Marker r_marker;
        r_marker.header.frame_id = "odom";
        r_marker.header.stamp    = now;
        r_marker.ns              = "rune_visualization";
        r_marker.id              = 5;
        r_marker.type            = visualization_msgs::msg::Marker::SPHERE;
        r_marker.action          = visualization_msgs::msg::Marker::ADD;

        r_marker.pose.position.x = r_center_world.x();
        r_marker.pose.position.y = r_center_world.y();
        r_marker.pose.position.z = r_center_world.z();
        r_marker.scale.x         = 0.1;
        r_marker.scale.y         = 0.1;
        r_marker.scale.z         = 0.1;
        r_marker.color.r         = 0.8;
        r_marker.color.g         = 0.8;
        r_marker.color.b         = 0.8;
        r_marker.color.a         = 1.0;
        r_marker.lifetime        = rclcpp::Duration::from_seconds(0.1);
        return r_marker;
    }

    visualization_msgs::msg::Marker create_target_marker(
        const rclcpp::Time& now, const Eigen::Vector3d& center, const Eigen::Quaterniond& q,
        double scale_x, double scale_y, double scale_z, double r, double g, double b, double a,
        int id) {

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id    = "odom";
        marker.header.stamp       = now;
        marker.ns                 = "rune_visualization";
        marker.id                 = id;
        marker.type               = visualization_msgs::msg::Marker::CUBE;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x    = center.x();
        marker.pose.position.y    = center.y();
        marker.pose.position.z    = center.z();
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x            = scale_x;
        marker.scale.y            = scale_y;
        marker.scale.z            = scale_z;
        marker.color.r            = r;
        marker.color.g            = g;
        marker.color.b            = b;
        marker.color.a            = a;
        marker.lifetime           = rclcpp::Duration::from_seconds(0.1);
        return marker;
    }
};
} // namespace fyt::rune
#endif // RUNE_DETECTOR_RUNE_DETECTOR_NODE_HPP_
