// STD
#include <thread>

// 3rdlibs
#include <opencv2/opencv.hpp>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace armor_auto_aim {
class PubVideoNode: public rclcpp::Node {
public:
    explicit PubVideoNode(const rclcpp::NodeOptions& options);
private:
    image_transport::Publisher m_img_pub;
    std::string m_filename;
    std::thread m_video_pub_thread;

    void run();    
};
}
