#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rm_calibration/extrinsic_calibrator.hpp"
#include "rm_calibration/intrinsic_calibrator.hpp"
#include "rm_calibration/types.hpp"

namespace rm_calibration {
namespace {

template <typename T>
T declare_or_get(rclcpp::Node* node, const std::string& name, const T& default_value) {
    try {
        return node->declare_parameter<T>(name, default_value);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
        T value{};
        (void)node->get_parameter(name, value);
        return value;
    }
}

int declare_or_get_int(rclcpp::Node* node, const std::string& name, int default_value) {
    const auto value64 = declare_or_get<int64_t>(node, name, static_cast<int64_t>(default_value));
    return static_cast<int>(value64);
}

std::filesystem::path expand_user(const std::string& path) {
    if (path.empty() || path[0] != '~') {
        return std::filesystem::path(path);
    }

    const char* home = std::getenv("HOME");
    if (home == nullptr) {
        return std::filesystem::path(path);
    }

    if (path.size() == 1U) {
        return std::filesystem::path(home);
    }

    if (path[1] == '/') {
        return std::filesystem::path(home) / path.substr(2);
    }

    return std::filesystem::path(path);
}

int compute_next_index(const std::filesystem::path& dir) {
    int max_index = 0;
    if (!std::filesystem::exists(dir)) {
        return 1;
    }

    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto stem = entry.path().stem().string();
        try {
            const int i = std::stoi(stem);
            if (i > max_index) {
                max_index = i;
            }
        } catch (...) {
            continue;
        }
    }
    return max_index + 1;
}

std::string zero_padded(int value, int width) {
    std::ostringstream ss;
    ss << std::setw(width) << std::setfill('0') << value;
    return ss.str();
}

std::vector<std::filesystem::path> list_images(const std::filesystem::path& dir) {
    std::vector<std::filesystem::path> images;
    if (!std::filesystem::exists(dir)) {
        return images;
    }

    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto ext = entry.path().extension().string();
        if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") {
            images.push_back(entry.path());
        }
    }

    std::sort(images.begin(), images.end(), [](const auto& a, const auto& b) {
        const auto sa = a.stem().string();
        const auto sb = b.stem().string();
        try {
            return std::stoi(sa) < std::stoi(sb);
        } catch (...) {
            return a.string() < b.string();
        }
    });

    return images;
}

std::vector<SampleFile> build_samples(const std::filesystem::path& dir) {
    std::vector<SampleFile> samples;
    for (const auto& img : list_images(dir)) {
        const auto tf_path = img.parent_path() / (img.stem().string() + ".txt");
        if (!std::filesystem::exists(tf_path)) {
            continue;
        }
        samples.push_back(SampleFile{img, tf_path});
    }
    return samples;
}

std::string format_intrinsic_yaml(const IntrinsicCalibrationResult& result) {
    auto mat_to_flow = [](const cv::Mat& m) {
        std::ostringstream ss;
        ss.setf(std::ios::fixed);
        ss << std::setprecision(12);
        ss << "[";
        bool first = true;
        for (int r = 0; r < m.rows; r++) {
            for (int c = 0; c < m.cols; c++) {
                if (!first) {
                    ss << ", ";
                }
                first = false;
                ss << m.at<double>(r, c);
            }
        }
        ss << "]";
        return ss.str();
    };

    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setprecision(12);
    ss << "# 重投影误差: " << result.reprojection_error_px << "px\n";
    ss << "# 使用样本数: " << result.used_samples << "\n";
    ss << "camera_matrix: " << mat_to_flow(result.camera_matrix) << "\n";
    ss << "distort_coeffs: " << mat_to_flow(result.dist_coeffs.reshape(1, 1)) << "\n";
    return ss.str();
}

bool camera_info_to_cv(
    const sensor_msgs::msg::CameraInfo& msg, cv::Mat& camera_matrix, cv::Mat& dist) {
    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    for (int i = 0; i < 9; i++) {
        camera_matrix.at<double>(i / 3, i % 3) = msg.k[static_cast<std::size_t>(i)];
    }

    dist = cv::Mat::zeros(static_cast<int>(msg.d.size()), 1, CV_64F);
    for (std::size_t i = 0; i < msg.d.size(); i++) {
        dist.at<double>(static_cast<int>(i), 0) = msg.d[i];
    }

    return true;
}

bool write_text_file(const std::filesystem::path& path, const std::string& content) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return false;
    }
    out << content;
    return static_cast<bool>(out);
}

} // namespace

class CalibrationNode final : public rclcpp::Node {
public:
    explicit CalibrationNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("calibration_node", options) {
        image_topic_       = declare_or_get<std::string>(this, "image_topic", "/image_raw");
        camera_info_topic_ = declare_or_get<std::string>(this, "camera_info_topic", "/camera_info");
        base_frame_        = declare_or_get<std::string>(this, "base_frame", "odom");

        gimbal_frame_     = declare_or_get<std::string>(this, "gimbal_frame", "");
        legacy_imu_frame_ = declare_or_get<std::string>(this, "imu_frame", "gimbal_link");
        if (gimbal_frame_.empty()) {
            gimbal_frame_ = legacy_imu_frame_;
        }

        data_dir_     = declare_or_get<std::string>(this, "data_dir", "~/rm_calibration_data");
        image_ext_    = declare_or_get<std::string>(this, "image_ext", "jpg");
        jpeg_quality_ = declare_or_get_int(this, "jpeg_quality", 95);
        png_level_    = declare_or_get_int(this, "png_level", 3);

        capture_timeout_s_  = declare_or_get<double>(this, "capture_timeout_s", 2.0);
        tf_timeout_s_       = declare_or_get<double>(this, "tf_timeout_s", 0.2);
        max_tf_time_diff_s_ = declare_or_get<double>(this, "max_tf_time_diff", 0.3);

        fix_k3_                  = declare_or_get<bool>(this, "fix_k3", true);
        use_gripper_translation_ = declare_or_get<bool>(this, "use_gripper_translation", true);

        intrinsic_output_path_ = declare_or_get<std::string>(this, "intrinsic_output_path", "");
        extrinsic_output_path_ = declare_or_get<std::string>(this, "extrinsic_output_path", "");

        mode_          = declare_or_get<std::string>(this, "mode", "extrinsic");
        board_width_   = declare_or_get_int(this, "board_width", 10);
        board_height_  = declare_or_get_int(this, "board_height", 7);
        square_size_m_ = declare_or_get<double>(this, "square_size", 0.015);

        data_dir_path_ = expand_user(data_dir_);
        std::filesystem::create_directories(data_dir_path_);
        next_index_.store(compute_next_index(data_dir_path_));

        tf_buffer_           = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Use Reentrant callback group for image subscriber to prevent blocking
        sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        srv_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sub_opt;
        sub_opt.callback_group = sub_cb_group_;

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, rclcpp::SensorDataQoS(),
            std::bind(&CalibrationNode::on_camera_info, this, std::placeholders::_1), sub_opt);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, rclcpp::SensorDataQoS(),
            std::bind(&CalibrationNode::on_image, this, std::placeholders::_1), sub_opt);

        capture_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "capture",
            std::bind(
                &CalibrationNode::on_capture, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, srv_cb_group_);

        calibrate_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "calibrate",
            std::bind(
                &CalibrationNode::on_calibrate, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, srv_cb_group_);

        RCLCPP_INFO(
            this->get_logger(),
            "rm_calibration ready: image_topic=%s camera_info_topic=%s data_dir=%s",
            image_topic_.c_str(), camera_info_topic_.c_str(), data_dir_path_.string().c_str());
        RCLCPP_INFO(
            this->get_logger(), "TF: base_frame=%s gimbal_frame=%s", base_frame_.c_str(),
            gimbal_frame_.c_str());
    }

private:
    struct CaptureResult {
        bool success = false;
        std::string message;
    };

    void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(camera_info_mutex_);
        last_camera_info_ = *msg;
        has_camera_info_  = true;
    }

    std::optional<geometry_msgs::msg::TransformStamped>
        lookup_gripper2base(const rclcpp::Time& stamp) {
        const auto tf_timeout = rclcpp::Duration::from_seconds(tf_timeout_s_);
        try {
            return tf_buffer_->lookupTransform(base_frame_, gimbal_frame_, stamp, tf_timeout);
        } catch (const tf2::TransformException&) {
            // Fallback: try latest transform if it is close enough.
        }

        try {
            auto tf = tf_buffer_->lookupTransform(
                base_frame_, gimbal_frame_, tf2::TimePointZero,
                tf2::durationFromSec(tf_timeout_s_));
            const rclcpp::Time tf_stamp(tf.header.stamp);
            const double dt = std::abs((stamp - tf_stamp).seconds());
            if (dt > max_tf_time_diff_s_) {
                return std::nullopt;
            }
            return tf;
        } catch (const tf2::TransformException&) {
            return std::nullopt;
        }
    }

    bool write_gripper2base_txt(
        const std::filesystem::path& path, const geometry_msgs::msg::TransformStamped& tf) {
        std::ostringstream ss;
        ss.setf(std::ios::fixed);
        ss << std::setprecision(17);

        ss << "# target_frame: " << tf.header.frame_id << "\n";
        ss << "# source_frame: " << tf.child_frame_id << "\n";
        ss << "# stamp: " << tf.header.stamp.sec << "." << tf.header.stamp.nanosec << "\n";

        const auto& t = tf.transform.translation;
        const auto& q = tf.transform.rotation;
        ss << t.x << " " << t.y << " " << t.z << " " << q.w << " " << q.x << " " << q.y << " "
           << q.z << "\n";

        return write_text_file(path, ss.str());
    }

    void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        // Lightweight callback - just store image data, don't do any disk I/O
        bool should_store = false;
        {
            std::lock_guard<std::mutex> lock(capture_mutex_);
            image_seq_++;
            if (capture_pending_ && !capture_in_process_ && image_seq_ > capture_from_seq_) {
                capture_in_process_ = true;
                should_store        = true;
            }
        }

        if (!should_store) {
            return;
        }

        // Copy image data (fast) - don't do disk I/O here
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (const cv_bridge::Exception& e) {
            std::lock_guard<std::mutex> lock(capture_mutex_);
            capture_pending_    = false;
            capture_in_process_ = false;
            capture_done_       = true;
            capture_success_    = false;
            capture_message_    = std::string("cv_bridge: ") + e.what();
            capture_cv_.notify_all();
            return;
        }

        // Store image copy and timestamp for processing in service callback
        {
            std::lock_guard<std::mutex> lock(capture_mutex_);
            pending_image_       = cv_ptr->image.clone();
            pending_image_stamp_ = rclcpp::Time(msg->header.stamp);
            capture_image_ready_ = true;
        }
        capture_cv_.notify_all();
    }

    void on_capture(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        const std_srvs::srv::Trigger::Response::SharedPtr response) {
        std::unique_lock<std::mutex> lock(capture_mutex_);
        if (capture_pending_ || capture_in_process_) {
            response->success = false;
            response->message = "capture already in progress";
            return;
        }

        capture_pending_     = true;
        capture_in_process_  = false;
        capture_done_        = false;
        capture_success_     = false;
        capture_image_ready_ = false;
        capture_message_.clear();
        capture_from_seq_ = image_seq_;

        const auto timeout = std::chrono::duration<double>(capture_timeout_s_);
        const auto deadline =
            std::chrono::steady_clock::now()
            + std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeout);

        // Wait for image to be ready (not for full capture completion)
        const bool ok =
            capture_cv_.wait_until(lock, deadline, [this]() { return capture_image_ready_; });

        if (!ok) {
            capture_pending_    = false;
            capture_in_process_ = false;
            response->success   = false;
            response->message   = "timeout waiting for next image";
            return;
        }

        // Image is ready, copy data out of mutex scope
        cv::Mat image_copy        = pending_image_.clone();
        rclcpp::Time image_stamp  = pending_image_stamp_;
        capture_image_ready_      = false;
        lock.unlock();

        // Do TF lookup and disk I/O outside of image callback (non-blocking for subscriber)
        CaptureResult result = process_captured_image(image_copy, image_stamp);

        // Update state
        lock.lock();
        capture_pending_    = false;
        capture_in_process_ = false;
        capture_done_       = true;
        capture_success_    = result.success;
        capture_message_    = result.message;
        lock.unlock();

        response->success = result.success;
        response->message = result.message;
    }

    CaptureResult process_captured_image(const cv::Mat& image, const rclcpp::Time& stamp) {
        CaptureResult res;

        const auto tf_opt = lookup_gripper2base(stamp);
        if (!tf_opt) {
            res.success = false;
            res.message = "tf lookup failed (base_frame=" + base_frame_
                        + " gimbal_frame=" + gimbal_frame_ + ")";
            return res;
        }

        const int index        = next_index_.fetch_add(1);
        const std::string stem = zero_padded(index, 6);

        const auto img_path = data_dir_path_ / (stem + "." + image_ext_);
        const auto tf_path  = data_dir_path_ / (stem + ".txt");

        std::vector<int> imwrite_params;
        if (image_ext_ == "jpg" || image_ext_ == "jpeg") {
            imwrite_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        } else if (image_ext_ == "png") {
            imwrite_params = {cv::IMWRITE_PNG_COMPRESSION, png_level_};
        }

        if (!cv::imwrite(img_path.string(), image, imwrite_params)) {
            res.success = false;
            res.message = "failed to write image: " + img_path.string();
            return res;
        }

        if (!write_gripper2base_txt(tf_path, *tf_opt)) {
            res.success = false;
            res.message = "failed to write tf: " + tf_path.string();
            return res;
        }

        res.success = true;
        res.message = "saved: " + img_path.string() + " and " + tf_path.string();
        return res;
    }

    void on_calibrate(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        const std_srvs::srv::Trigger::Response::SharedPtr response) {
        const std::string mode = this->get_parameter("mode").as_string();

        const auto data_dir_path = expand_user(this->get_parameter("data_dir").as_string());
        if (!std::filesystem::exists(data_dir_path)) {
            response->success = false;
            response->message = "data_dir not found: " + data_dir_path.string();
            return;
        }

        const int board_width      = this->get_parameter("board_width").as_int();
        const int board_height     = this->get_parameter("board_height").as_int();
        const double square_size_m = this->get_parameter("square_size").as_double();
        if (board_width <= 0 || board_height <= 0 || square_size_m <= 0.0) {
            response->success = false;
            response->message = "invalid board parameters";
            return;
        }

        if (mode == "intrinsic") {
            const auto images = list_images(data_dir_path);
            IntrinsicCalibrator calibrator(cv::Size(board_width, board_height), square_size_m);
            const bool fix_k3 = this->get_parameter("fix_k3").as_bool();
            const auto result = calibrator.calibrate(images, fix_k3);

            if (result.camera_matrix.empty() || result.dist_coeffs.empty()
                || result.used_samples < 3U) {
                response->success = false;
                response->message = "intrinsic calibration failed: valid_samples="
                                  + std::to_string(result.used_samples)
                                  + " (need >= 3). "
                                    "Check board_width/board_height and pattern type.";
                return;
            }

            std::filesystem::path out_path;
            const auto out_param = this->get_parameter("intrinsic_output_path").as_string();
            if (!out_param.empty()) {
                out_path = expand_user(out_param);
            } else {
                out_path = data_dir_path / "intrinsic.yaml";
            }

            if (!write_text_file(out_path, format_intrinsic_yaml(result))) {
                response->success = false;
                response->message = "failed to write: " + out_path.string();
                return;
            }

            response->success = true;
            response->message =
                "intrinsic done, reproj_error_px=" + std::to_string(result.reprojection_error_px)
                + ", yaml=" + out_path.string();
            return;
        }

        if (mode == "extrinsic") {
            sensor_msgs::msg::CameraInfo cam_info;
            {
                std::lock_guard<std::mutex> lock(camera_info_mutex_);
                if (!has_camera_info_) {
                    response->success = false;
                    response->message = "no /camera_info received yet";
                    return;
                }
                cam_info = last_camera_info_;
            }

            cv::Mat camera_matrix, dist_coeffs;
            camera_info_to_cv(cam_info, camera_matrix, dist_coeffs);

            const auto samples = build_samples(data_dir_path);
            ExtrinsicCalibrator calibrator(cv::Size(board_width, board_height), square_size_m);
            const bool use_translation = this->get_parameter("use_gripper_translation").as_bool();
            const auto result =
                calibrator.calibrate(samples, camera_matrix, dist_coeffs, use_translation);

            if (result.R_camera2gimbal.empty() || result.t_camera2gimbal.empty()
                || result.used_samples < 3U) {
                response->success = false;
                response->message = "extrinsic calibration failed: valid_samples="
                                  + std::to_string(result.used_samples)
                                  + " (need >= 3). "
                                    "Most common cause: board detection failed (pattern mismatch "
                                    "or wrong board size).";
                return;
            }

            std::filesystem::path out_path;
            const auto out_param = this->get_parameter("extrinsic_output_path").as_string();
            if (!out_param.empty()) {
                out_path = expand_user(out_param);
            } else {
                out_path = data_dir_path / "extrinsic.yaml";
            }

            const auto yaml =
                ExtrinsicCalibrator::format_extrinsic_yaml(result, base_frame_, gimbal_frame_);
            if (!write_text_file(out_path, yaml)) {
                response->success = false;
                response->message = "failed to write: " + out_path.string();
                return;
            }

            response->success = true;
            response->message = "extrinsic done, yaml=" + out_path.string();
            return;
        }

        response->success = false;
        response->message = "unknown mode: " + mode + " (use 'intrinsic' or 'extrinsic')";
    }

    std::string image_topic_;
    std::string camera_info_topic_;
    std::string base_frame_;
    std::string gimbal_frame_;
    std::string legacy_imu_frame_;

    std::string data_dir_;
    std::filesystem::path data_dir_path_;
    std::string image_ext_;
    int jpeg_quality_ = 95;
    int png_level_    = 3;

    double capture_timeout_s_  = 2.0;
    double tf_timeout_s_       = 0.2;
    double max_tf_time_diff_s_ = 0.3;

    bool fix_k3_                  = true;
    bool use_gripper_translation_ = true;

    std::string intrinsic_output_path_;
    std::string extrinsic_output_path_;

    std::string mode_;
    int board_width_      = 10;
    int board_height_     = 7;
    double square_size_m_ = 0.015;

    std::atomic<int> next_index_{1};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
    rclcpp::CallbackGroup::SharedPtr srv_cb_group_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;

    std::mutex camera_info_mutex_;
    bool has_camera_info_ = false;
    sensor_msgs::msg::CameraInfo last_camera_info_;

    std::mutex capture_mutex_;
    std::condition_variable capture_cv_;
    bool capture_pending_    = false;
    bool capture_in_process_ = false;
    bool capture_done_       = false;
    bool capture_success_    = false;
    bool capture_image_ready_ = false;
    cv::Mat pending_image_;
    rclcpp::Time pending_image_stamp_;
    std::string capture_message_;
    std::uint64_t image_seq_        = 0;
    std::uint64_t capture_from_seq_ = 0;
};

} // namespace rm_calibration

#ifndef RM_CALIBRATION_COMPONENT
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node    = std::make_shared<rm_calibration::CalibrationNode>(options);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
#endif

#ifdef RM_CALIBRATION_COMPONENT
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rm_calibration::CalibrationNode)
#endif
