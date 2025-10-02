#pragma once
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "quality_gate.hpp"

namespace rm_calibration {

struct Frame {
    cv::Mat image;   
    Eigen::Quaterniond imu_q; // imu_abs 四元数（w,x,y,z）
};

enum class PatternType { SymmetricCircles, Chessboard };

class Calibrator {
public:
    Calibrator();
    ~Calibrator() = default;

    // ---- 配置接口 ----
    void setPattern(
        int cols, int rows, double spacing_mm, PatternType type = PatternType::Chessboard);
    void setCameraIntrinsics(
        const cv::Mat& K, const cv::Mat& D); // 若未设置，将在 calibrateCameraFromBuffer() 中估计
    void setRGimbal2ImuBody(const Eigen::Matrix3d& R_g2ib); // 可选：提供云台->IMU安装姿态
    void setOutputYamlPath(const std::string& path);        // 默认 handeye.yaml

    // ---- 数据接口：推入一帧图像 + 同步IMU姿态 ----
    // 注意：内部会复制图像，如需零拷贝可改成 cv::Mat::clone() 控制
    void pushFrame(const cv::Mat& img_bgr, const Eigen::Quaterniond& imu_q);

    // ---- 标定入口（处理缓冲区）----
    // 1) 仅相机内参（根据缓冲区的角点检测）
    // 返回 reprojection error（像素）
    double calibrateCameraFromBuffer();

    // 2) 手眼（经典）：输出 camera->gimbal
    // 要求：缓冲区能检测出足够角点；IMU 四元数充足且姿态覆盖多样
    bool calibrateHandEye(cv::Mat& rotation_camera2gimbal, cv::Mat& translation_camera2gimbal_m);

    // 4) 写 YAML（工具）
    bool writeYaml(
        const cv::Mat& R_camera2gimbal, const cv::Mat& translation_camera2gimbal_m,
        const std::optional<cv::Mat>& rotation_world2board      = std::nullopt,
        const std::optional<cv::Mat>& translation_world2board_m = std::nullopt) const;

    void setQualityParams(const QualityParams& p) { qp_ = p; }

    // ---- 辅助：获取已收集样本数 ----
    size_t sampleCount() const;

private:
    // --- private: 新增成员 ---
    QualityParams qp_;
    // 内部工具
    bool detectPattern(const cv::Mat& img, std::vector<cv::Point2f>& img_points) const;
    void buildPlanarObjectPoints(std::vector<cv::Point3f>& obj_pts) const;
    static cv::Mat eigenRToCv(const Eigen::Matrix3d& R);

    // 从缓冲区生成 HandEye / RobotWorldHandEye 的输入序列
    // 返回有效样本数
    size_t buildPnPAndRobotMotions(
        std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
        std::vector<cv::Mat>& rotation_gimbal2world_list,
        std::vector<cv::Mat>& translation_gimbal2world_list) const;

private:
    // 配置
    int cols_             = 9;   // 列
    int rows_             = 6;    // 行
    double spacing_mm_    = 20.0; // 圆心距 / 格子边长（mm）
    PatternType pattern_  = PatternType::SymmetricCircles;
    std::string yaml_out_ = "handeye.yaml";

    // 相机内参
    bool has_intrinsics_ = false;
    cv::Mat K_, D_; // CV_64F

    // 云台->IMU 安装姿态（可选，不设则当作单位阵）
    Eigen::Matrix3d R_gimbal2imubody_ = Eigen::Matrix3d::Identity();

    // 缓冲区
    std::vector<Frame> buf_;
    mutable std::mutex mtx_;
};

} // namespace rm_calibration
