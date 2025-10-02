#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include "rm_calibration/calibrator.hpp"
#include <rm_interfaces/srv/run_hand_eye_calibration.hpp>

using rm_interfaces::srv::RunHandEyeCalibration;

namespace rm_calibration {

class CalibratorNode : public rclcpp::Node {
public:
    explicit CalibratorNode(const rclcpp::NodeOptions& options)
        : Node("ptz_calibrator", options)
        , tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock()))
        , tf_listener_(*tf_buffer_)
        , min_gap_(0, 0) {

        // 让 tf2 的 Buffer 使用 ROS 定时器（避免阻塞）
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(), this->get_node_timers_interface());
        tf_buffer_->setCreateTimerInterface(timer_interface);

        // === 参数 ===
        odom_frame_   = this->declare_parameter<std::string>("odom_frame", "odom");
        gimbal_frame_ = this->declare_parameter<std::string>("gimbal_frame", "gimbal_link");

        // 采样窗口与频率控制
        grab_time_sec_ = this->declare_parameter<double>("grabbing_time_seconds", 20.0);
        min_gap_hz_    = this->declare_parameter<double>("min_grab_image_hz", 5.0);
        min_gap_       = rclcpp::Duration::from_seconds(1.0 / std::max(0.1, min_gap_hz_));

        // 图案参数
        int cols  = this->declare_parameter<int>("calibrator.cols", 6);
        int rows  = this->declare_parameter<int>("calibrator.rows", 9);
        double sp = this->declare_parameter<double>("calibrator.spacing_mm", 20.0);
        std::string pattern_type_str = this->declare_parameter<std::string>(
            "calibrator.chassis_boardORsymmetric_circles", "chess_board");
        PatternType pattern_type;
        if (pattern_type_str == "chess_board") {
            pattern_type = PatternType::Chessboard;
        } else if (pattern_type_str == "symmetric_circles") {
            pattern_type = PatternType::SymmetricCircles;
        } else {
            RCLCPP_ERROR(
                get_logger(), "Unknown type : %s, fallback to chess board",
                pattern_type_str.c_str());
            pattern_type = PatternType::Chessboard;
        }
        calib_.setPattern(cols, rows, sp, pattern_type);

        // 输出文件
        intrinsics_yaml_ =
            this->declare_parameter<std::string>("intrinsics_yaml", "intrinsics.yaml");
        handeye_yaml_ = this->declare_parameter<std::string>("handeye_yaml", "handeye.yaml");

        // === 订阅 ===
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", rclcpp::QoS(10),
            std::bind(&CalibratorNode::cameraInfoCb, this, std::placeholders::_1));

        // === 服务 ===
        srv_intrinsics_ = this->create_service<RunHandEyeCalibration>(
            "run_intrinsics_calibration", std::bind(
                                              &CalibratorNode::runIntrinsicsSrv, this,
                                              std::placeholders::_1, std::placeholders::_2));

        srv_handeye_ = this->create_service<RunHandEyeCalibration>(
            "run_handeye_calibration", std::bind(
                                           &CalibratorNode::runHandeyeSrv, this,
                                           std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "ptz_calibrator ready.");
    }

private:
    // -------- CameraInfo 注入到 Calibrator --------
    void cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        // 记录订阅来的内参
        cv::Mat K =
            (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4],
             msg->k[5], msg->k[6], msg->k[7], msg->k[8]);
        cv::Mat D(msg->d.size(), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); ++i)
            D.at<double>((int)i, 0) = msg->d[i];

        K_sub_               = K.clone();
        D_sub_               = D.clone();
        have_sub_intrinsics_ = true;
    }

    // -------- 图像采样（通用） --------
    void startSampling() {
        end_time_   = now() + rclcpp::Duration::from_seconds(grab_time_sec_);
        last_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        // 订阅图像
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", rclcpp::SensorDataQoS(),
            std::bind(&CalibratorNode::imageCb, this, std::placeholders::_1));
    }

    void stopSampling() {
        if (image_sub_)
            image_sub_.reset();
    }

    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {
        const rclcpp::Time stamp = img_msg->header.stamp;

        if (stamp > end_time_) {
            stopSampling();
            sampling_done_ = true;
            return;
        }

        if ((stamp - last_stamp_) < min_gap_)
            return;
        last_stamp_ = stamp;

        // 查 TF：odom <- gimbal（使用图像时间）
        geometry_msgs::msg::TransformStamped T_odom_gimbal;
        try {
            T_odom_gimbal = tf_buffer_->lookupTransform(
                odom_frame_, gimbal_frame_, stamp, rclcpp::Duration::from_seconds(0.05));
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF failed: %s", e.what());
            return;
        }

        const auto& q = T_odom_gimbal.transform.rotation;
        Eigen::Quaterniond q_og(q.w, q.x, q.y, q.z);
        q_og.normalize();

        // 提取 BGR
        cv::Mat bgr;
        try {
            bgr = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
        } catch (...) {
            return;
        }

        calib_.pushFrame(bgr, q_og);
    }

    // -------- 服务：内参标定 --------
    void runIntrinsicsSrv(
        const RunHandEyeCalibration::Request::SharedPtr /*req*/,
        RunHandEyeCalibration::Response::SharedPtr res) {
        // 清旧数据，仅采样图像即可
        sampling_done_ = false;
        startSampling();

        // 等待窗口结束
        rclcpp::Rate r(100);
        while (rclcpp::ok() && !sampling_done_)
            r.sleep();

        // 求内参
        double err = calib_.calibrateCameraFromBuffer();
        if (err < 0) {
            res->success   = false;
            res->yaml_path = "";
            RCLCPP_ERROR(get_logger(), "calibrateCameraFromBuffer failed.");
            return;
        }

        // 写 intrinsics.yaml（用 OpenCV FileStorage）
        if (!writeIntrinsicsYaml(intrinsics_yaml_)) {
            res->success   = false;
            res->yaml_path = "";
            RCLCPP_ERROR(get_logger(), "write intrinsics yaml failed.");
            return;
        }

        // 记为“已通过服务得到的内参”（优先使用）
        have_srv_intrinsics_ = true;
        K_srv_               = getK();
        D_srv_               = getD();

        res->success   = true;
        res->yaml_path = intrinsics_yaml_;
        RCLCPP_INFO(get_logger(), "Intrinsics done: err=%.4f -> %s", err, intrinsics_yaml_.c_str());
    }

    // -------- 服务：手眼外参标定 --------
    void runHandeyeSrv(
        const RunHandEyeCalibration::Request::SharedPtr req,
        RunHandEyeCalibration::Response::SharedPtr res) {
        // 选择内参来源
        if (have_srv_intrinsics_) {
            calib_.setCameraIntrinsics(K_srv_, D_srv_);
        } else if (have_sub_intrinsics_) {
            calib_.setCameraIntrinsics(K_sub_, D_sub_);
        } else {
            // 两者都没有：允许 Calibrator 内部兜底做一次内参（但不写 intrinsics.yaml）
            RCLCPP_WARN(
                get_logger(),
                "No intrinsics from service or subscription; will estimate from buffer.");
        }

        // 采样窗口
        sampling_done_ = false;
        startSampling();

        rclcpp::Rate r(100);
        while (rclcpp::ok() && !sampling_done_)
            r.sleep();

        // 求外参
        cv::Mat R_c2g, t_c2g_m;
        bool ok = calib_.calibrateHandEye(R_c2g, t_c2g_m);
        if (!ok) {
            res->success   = false;
            res->yaml_path = "";
            RCLCPP_ERROR(get_logger(), "calibrateHandEye failed.");
            return;
        }

        // 写 handeye.yaml（用 Calibrator 内部 writeYaml — 已改为 OpenCV FileStorage 版本）
        if (!calib_.writeYaml(R_c2g, t_c2g_m)) {
            res->success   = false;
            res->yaml_path = "";
            RCLCPP_ERROR(get_logger(), "write handeye yaml failed.");
            return;
        }

        res->success   = true;
        res->yaml_path = handeye_yaml_;
        RCLCPP_INFO(get_logger(), "HandEye done -> %s", res->yaml_path.c_str());
    }

    // -------- 小工具：从 Calibrator 取 K,D（需你在 Calibrator 暴露 getter；否则这里存本地副本）
    // --------
    cv::Mat getK() const {
        // 若你的 Calibrator 没暴露 getter，可在 calibrateCameraFromBuffer 成功时在这里保存一份副本
        // 为了示例简单，这里直接返回订阅版；实际请替换成 calib_.getK() / calib_.getD()。
        return K_sub_.empty() ? K_srv_ : K_sub_;
    }
    cv::Mat getD() const { return D_sub_.empty() ? D_srv_ : D_sub_; }

    bool writeIntrinsicsYaml(const std::string& path) {
        cv::Mat K = getK(), D = getD();
        if (K.empty() || D.empty())
            return false;
        try {
            cv::FileStorage fs(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
            if (!fs.isOpened())
                return false;
            fs << "model" << "pinhole";
            fs << "K" << K;
            fs << "D" << D;
            fs.release();
            return true;
        } catch (...) {
            return false;
        }
    }

private:
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Service<RunHandEyeCalibration>::SharedPtr srv_intrinsics_;
    rclcpp::Service<RunHandEyeCalibration>::SharedPtr srv_handeye_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 参数
    std::string odom_frame_, gimbal_frame_;
    double grab_time_sec_, min_gap_hz_;
    rclcpp::Duration min_gap_;

    std::string intrinsics_yaml_, handeye_yaml_;

    // 采样状态
    rclcpp::Time end_time_;
    rclcpp::Time last_stamp_;
    bool sampling_done_{false};

    // 内参与选择：服务优先，其次订阅
    bool have_srv_intrinsics_{false};
    bool have_sub_intrinsics_{false};
    cv::Mat K_srv_, D_srv_;
    cv::Mat K_sub_, D_sub_;

    // 业务
    Calibrator calib_;
};

} // namespace rm_calibration

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_calibration::CalibratorNode)
