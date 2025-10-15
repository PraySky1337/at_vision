#include "pub_video/pub_video_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>

namespace pub_video {
PubVideoNode::PubVideoNode(const rclcpp::NodeOptions& options)
    : Node("pub_video_node", options)
    , tf_broadcaster_(*this) {

    camera_pub_ =
        image_transport::create_camera_publisher(this, "image_raw", rmw_qos_profile_sensor_data);
    tf_buffer_                 = std::make_shared<tf2_ros::Buffer>(get_clock());
    image_msg_.header.frame_id = "camera_optical_frame";
    image_msg_.encoding        = "rgb8";

    RCLCPP_INFO(this->get_logger(), "PubVideoNode started.");
    video_path_ = declare_parameter("video_path", "video.mp4");
    video_capture_.open(video_path_);
    double fps = video_capture_.get(cv::CAP_PROP_FPS);
    RCLCPP_INFO(this->get_logger(), "VideoCapture opened %s with fps=%f", video_path_.c_str(), fps);
    // Load camera info (unchanged)
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
        this->declare_parameter("camera_info_url", "package://rm_bringup/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
        camera_info_manager_->loadCameraInfo(camera_info_url);
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / fps)),
        std::bind(&PubVideoNode::timerCallback, this));
}

void PubVideoNode::timerCallback() {
    cv::Mat frame;
    if (!video_capture_.isOpened()) {
        RCLCPP_WARN(
            this->get_logger(), "VideoCapture not opened! try to open %s", video_path_.c_str());
        video_capture_.open(video_path_);
        return;
    }
    if (!video_capture_.read(frame)) {
        RCLCPP_WARN(this->get_logger(), "Failed to read frame from video source! Reopen it.");
        video_capture_.open(video_path_);
        return;
    }
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_optical_frame";
    camera_pub_.publish(*msg, camera_info_msg_);
    // 发布 I 到 tf
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp            = msg->header.stamp;
    t.header.frame_id         = "odom";
    t.child_frame_id          = "gimbal_link";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x    = 0.0;
    t.transform.rotation.y    = 0.0;
    t.transform.rotation.z    = 0.0;
    t.transform.rotation.w    = 1.0;
    tf_broadcaster_.sendTransform(t);
}

} // namespace pub_video

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pub_video::PubVideoNode)
