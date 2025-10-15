// STD
#include <thread>

// 3rdlibs
#include <opencv2/opencv.hpp>
// ROS2
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <camera_info_manager/camera_info_manager.hpp>

namespace pub_video {
class PubVideoNode : public rclcpp::Node {
public:
    explicit PubVideoNode(const rclcpp::NodeOptions& options);

private:
    void timerCallback();

private:
    std::string video_path_;
    cv::VideoCapture video_capture_;
    tf2_ros::Buffer::SharedPtr tf_buffer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    sensor_msgs::msg::Image image_msg_;
    image_transport::CameraPublisher camera_pub_;
    // Camera info manager
    std::string camera_name_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
};
} // namespace pub_video
