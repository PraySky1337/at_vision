#include <pub_video/pub_video_node.hpp>

#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace armor_auto_aim {
PubVideoNode::PubVideoNode(const rclcpp::NodeOptions& options)
    : Node("pub_video", options) {
    m_filename = this->declare_parameter("video_filename", "/video/blue.MP4");
    auto pkg_path = ament_index_cpp::get_package_share_directory("pub_video");
    m_filename = pkg_path + m_filename;
    // m_filename
    m_img_pub = image_transport::create_publisher(this, "/image_raw");

    m_video_pub_thread = std::thread(&PubVideoNode::run, this);
    m_video_pub_thread.detach();
}

void PubVideoNode::run() {
    RCLCPP_INFO(this->get_logger(), "video starting...");
    cv::VideoCapture capture(m_filename);
    cv::Mat frame;
    cv::Mat scaled_frame;
    if (!capture.isOpened()) {
        RCLCPP_WARN(this->get_logger(), "invalid filename: %s", m_filename.c_str());
        return;
    }
    while (rclcpp::ok()) {
        capture >> frame;
        if (!frame.empty()) {
            cv::resize(frame, scaled_frame, cv::Size(frame.cols*0.5, frame.rows*0.5));
            std_msgs::msg::Header header;
            header.stamp = this->now();
            m_img_pub.publish(*(cv_bridge::CvImage(header, "bgr8", scaled_frame).toImageMsg()));
            cv::waitKey(1000 / capture.get(cv::CAP_PROP_FPS));
            // cv::waitKey(500);
            // cv::waitKey(1);
        }
    }
    RCLCPP_INFO(this->get_logger(), "video_finished!");
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::PubVideoNode)
