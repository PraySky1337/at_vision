#include "rm_calibration/extrinsic_calibrator.hpp"

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <iomanip>
#include <sstream>

namespace rm_calibration {
namespace {

constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

Eigen::Vector3d yxz_euler(const Eigen::Matrix3d& R) {
    // Decompose R = Ry(yaw) * Rx(pitch) * Rz(roll)
    // Return (yaw, pitch, roll) in radians.
    const double r12   = R(1, 2);
    const double sx    = std::clamp(-r12, -1.0, 1.0);
    const double pitch = std::asin(sx);
    const double cx    = std::cos(pitch);

    double yaw  = 0.0;
    double roll = 0.0;

    if (std::abs(cx) > 1e-9) {
        yaw  = std::atan2(R(0, 2), R(2, 2));
        roll = std::atan2(R(1, 0), R(1, 1));
    } else {
        // Gimbal lock: choose roll=0 and solve yaw from remaining terms.
        yaw  = std::atan2(-R(2, 0), R(0, 0));
        roll = 0.0;
    }

    return {yaw, pitch, roll};
}

// Extract roll, pitch, yaw from rotation matrix (XYZ Euler convention, i.e., R = Rz(yaw) * Ry(pitch) * Rx(roll))
Eigen::Vector3d xyz_euler_rpy(const Eigen::Matrix3d& R) {
    double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
    double cp    = std::cos(pitch);

    double roll, yaw;
    if (std::abs(cp) > 1e-9) {
        roll = std::atan2(R(2, 1), R(2, 2));
        yaw  = std::atan2(R(1, 0), R(0, 0));
    } else {
        // Gimbal lock
        roll = std::atan2(-R(1, 2), R(1, 1));
        yaw  = 0.0;
    }

    return {roll, pitch, yaw};
}

std::vector<double> mat_to_vector_rowmajor(const cv::Mat& m) {
    std::vector<double> v;
    if (m.empty()) {
        return v;
    }

    v.reserve(static_cast<std::size_t>(m.total()));
    for (int r = 0; r < m.rows; r++) {
        for (int c = 0; c < m.cols; c++) {
            v.push_back(m.at<double>(r, c));
        }
    }
    return v;
}

std::string format_flow(const std::vector<double>& vals) {
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setprecision(12);
    ss << "[";
    for (std::size_t i = 0; i < vals.size(); i++) {
        ss << vals[i];
        if (i + 1 < vals.size()) {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
}

} // namespace

ExtrinsicCalibrator::ExtrinsicCalibrator(cv::Size pattern_size, double center_distance_m)
    : pattern_size_(pattern_size)
    , center_distance_m_(center_distance_m) {}

std::vector<cv::Point3f> ExtrinsicCalibrator::make_object_points() const {
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(pattern_size_.width * pattern_size_.height));

    for (int i = 0; i < pattern_size_.height; i++) {
        for (int j = 0; j < pattern_size_.width; j++) {
            points.emplace_back(
                static_cast<float>(j * center_distance_m_),
                static_cast<float>(i * center_distance_m_), 0.0F);
        }
    }

    return points;
}

bool ExtrinsicCalibrator::detect(const cv::Mat& image, std::vector<cv::Point2f>& centers_2d) const {
    if (image.empty()) {
        return false;
    }

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    centers_2d.clear();
    if (cv::findCirclesGrid(gray, pattern_size_, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID)) {
        return true;
    }

    std::vector<cv::Point2f> corners;
    const bool ok = cv::findChessboardCorners(
        gray, pattern_size_, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    if (!ok) {
        return false;
    }

    cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    centers_2d = std::move(corners);
    return true;
}

bool ExtrinsicCalibrator::load_gripper2base_from_txt(
    const std::filesystem::path& path, Pose& out_pose) {
    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }

    std::vector<double> vals;
    vals.reserve(7);

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream ss(line);
        double v = 0.0;
        while (ss >> v) {
            vals.push_back(v);
        }

        if (vals.size() >= 7U) {
            break;
        }
    }

    if (vals.size() < 7U) {
        return false;
    }

    const double tx = vals[0];
    const double ty = vals[1];
    const double tz = vals[2];
    const double qw = vals[3];
    const double qx = vals[4];
    const double qy = vals[5];
    const double qz = vals[6];

    Eigen::Quaterniond q(qw, qx, qy, qz);
    if (q.norm() <= 0.0) {
        return false;
    }
    q.normalize();

    out_pose.R = q.toRotationMatrix();
    out_pose.t = Eigen::Vector3d(tx, ty, tz);
    return true;
}

ExtrinsicCalibrationResult ExtrinsicCalibrator::calibrate(
    const std::vector<SampleFile>& samples, const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs, bool use_gripper_translation) const {
    ExtrinsicCalibrationResult out;

    std::vector<cv::Mat> R_gripper2base_list;
    std::vector<cv::Mat> t_gripper2base_list;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    const auto obj_template = make_object_points();

    for (const auto& sample : samples) {
        const cv::Mat img = cv::imread(sample.image_path.string(), cv::IMREAD_COLOR);
        if (img.empty()) {
            continue;
        }

        Pose gripper2base;
        if (!load_gripper2base_from_txt(sample.tf_path, gripper2base)) {
            continue;
        }

        std::vector<cv::Point2f> centers_2d;
        const bool success = detect(img, centers_2d);
        if (!success) {
            continue;
        }

        cv::Mat rvec, tvec;
        const bool pnp_ok = cv::solvePnP(
            obj_template, centers_2d, camera_matrix, dist_coeffs, rvec, tvec, false,
            cv::SOLVEPNP_IPPE);
        if (!pnp_ok) {
            continue;
        }

        cv::Mat R_gripper2base_cv;
        cv::eigen2cv(gripper2base.R, R_gripper2base_cv);

        cv::Mat t_gripper2base_cv =
            (cv::Mat_<double>(3, 1) << gripper2base.t.x(), gripper2base.t.y(), gripper2base.t.z());
        if (!use_gripper_translation) {
            t_gripper2base_cv = cv::Mat::zeros(3, 1, CV_64F);
        }

        R_gripper2base_list.emplace_back(std::move(R_gripper2base_cv));
        t_gripper2base_list.emplace_back(std::move(t_gripper2base_cv));
        rvecs.emplace_back(std::move(rvec));
        tvecs.emplace_back(std::move(tvec));
    }

    if (rvecs.size() < 3U) {
        out.used_samples = rvecs.size();
        return out;
    }

    cv::calibrateHandEye(
        R_gripper2base_list, t_gripper2base_list, rvecs, tvecs, out.R_camera2gimbal,
        out.t_camera2gimbal);

    Eigen::Matrix3d R_camera2gimbal_eigen = Eigen::Matrix3d::Identity();
    cv::cv2eigen(out.R_camera2gimbal, R_camera2gimbal_eigen);

    Eigen::Vector3d t_camera2gimbal_eigen;
    t_camera2gimbal_eigen << out.t_camera2gimbal.at<double>(0, 0),
        out.t_camera2gimbal.at<double>(1, 0), out.t_camera2gimbal.at<double>(2, 0);

    // --- Compute URDF-compatible output ---
    // cv::calibrateHandEye gives: camera_optical_frame -> gimbal_link
    // URDF needs: gimbal_link -> camera_link (in ROS frame)
    //
    // ROS coordinate frame convention (REP-103): x-forward, y-left, z-up
    // Camera optical frame convention: x-right, y-down, z-forward
    //
    // From URDF: camera_link -> camera_optical_frame has rpy=(-π/2, 0, -π/2)
    // This means: R_ros_to_optical = Rz(-π/2) * Ry(0) * Rx(-π/2)

    // Build R_ros_to_optical (camera_link -> camera_optical_frame rotation)
    Eigen::Matrix3d R_ros_to_optical;
    // After applying Rx(-π/2) then Rz(-π/2):
    // x_optical =  y_ros
    // y_optical =  z_ros
    // z_optical =  x_ros
    // So: R_ros_to_optical maps [1,0,0] -> [0,0,1], [0,1,0] -> [1,0,0], [0,0,1] -> [0,1,0]
    R_ros_to_optical << 0, 1, 0, 0, 0, 1, 1, 0, 0;

    // T_gimbal_to_camera_optical = T_camera_optical_to_gimbal^(-1)
    // R_gimbal_to_camera_optical = R_camera2gimbal^T
    // t_gimbal_to_camera_optical = -R_camera2gimbal^T * t_camera2gimbal
    Eigen::Matrix3d R_gimbal_to_optical = R_camera2gimbal_eigen.transpose();
    Eigen::Vector3d t_gimbal_to_optical = -R_gimbal_to_optical * t_camera2gimbal_eigen;

    // T_gimbal_to_camera_link = T_gimbal_to_camera_optical * T_camera_optical_to_camera_link
    // where T_camera_optical_to_camera_link = T_camera_link_to_camera_optical^(-1)
    // R_optical_to_ros = R_ros_to_optical^T
    Eigen::Matrix3d R_optical_to_ros  = R_ros_to_optical.transpose();
    Eigen::Matrix3d R_gimbal_to_camera = R_gimbal_to_optical * R_optical_to_ros;

    // Translation: t_gimbal_to_camera = R_gimbal_to_optical * t_optical_to_ros + t_gimbal_to_optical
    // But t_optical_to_ros = 0 (same origin), so:
    Eigen::Vector3d t_gimbal_to_camera = t_gimbal_to_optical;

    // Extract URDF xyz and rpy
    out.urdf_xyz = t_gimbal_to_camera;
    out.urdf_rpy = xyz_euler_rpy(R_gimbal_to_camera);

    // Compute assembly error: deviation from ideal mounting
    // Ideal: camera_link is aligned with gimbal_link (R = I, t = some nominal offset)
    // Assembly error = actual rotation in human-readable form
    out.assembly_error_deg = out.urdf_rpy * kRad2Deg;

    // Also compute original misalignment metric
    Eigen::Matrix3d R_gimbal2ideal;
    R_gimbal2ideal << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    const Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
    const Eigen::Vector3d ypr_rad        = yxz_euler(R_camera2ideal);
    out.misalignment_ypr_deg             = ypr_rad * kRad2Deg;

    out.used_samples = rvecs.size();
    return out;
}

std::string ExtrinsicCalibrator::format_extrinsic_yaml(
    const ExtrinsicCalibrationResult& result, const std::string& base_frame,
    const std::string& gimbal_frame) {
    (void)base_frame;
    (void)gimbal_frame;

    std::ostringstream ss;
    ss.setf(std::ios::fixed);

    // Header with human-readable summary
    ss << "# ============================================================\n";
    ss << "# 外参标定结果 (Extrinsic Calibration Result)\n";
    ss << "# ============================================================\n";
    ss << "#\n";
    ss << "# 使用样本数: " << result.used_samples << "\n";
    ss << "#\n";

    // URDF-compatible output (what users actually need)
    ss << "# ============================================================\n";
    ss << "# URDF参数 (直接复制到urdf文件即可)\n";
    ss << "# gimbal_link -> camera_link 变换\n";
    ss << "# ============================================================\n";
    ss << std::setprecision(6);
    ss << "#\n";
    ss << "# camera_xyz: \"" << result.urdf_xyz.x() << " " << result.urdf_xyz.y() << " "
       << result.urdf_xyz.z() << "\"\n";
    ss << "# camera_rpy: \"" << result.urdf_rpy.x() << " " << result.urdf_rpy.y() << " "
       << result.urdf_rpy.z() << "\"\n";
    ss << "#\n";

    // Human-readable assembly error
    ss << "# ============================================================\n";
    ss << "# 装配偏差 (Assembly Error)\n";
    ss << "# ============================================================\n";
    ss << std::setprecision(3);
    ss << "#\n";
    ss << "# 平移偏差 (Translation Error):\n";
    ss << "#   X (前后): " << std::showpos << result.urdf_xyz.x() * 1000.0 << " mm"
       << (result.urdf_xyz.x() > 0 ? " (相机靠前)" : " (相机靠后)") << "\n";
    ss << "#   Y (左右): " << std::showpos << result.urdf_xyz.y() * 1000.0 << " mm"
       << (result.urdf_xyz.y() > 0 ? " (相机靠左)" : " (相机靠右)") << "\n";
    ss << "#   Z (上下): " << std::showpos << result.urdf_xyz.z() * 1000.0 << " mm"
       << (result.urdf_xyz.z() > 0 ? " (相机靠上)" : " (相机靠下)") << "\n";
    ss << std::noshowpos;
    ss << "#\n";
    ss << "# 旋转偏差 (Rotation Error):\n";
    ss << "#   Roll  (绕X轴): " << std::showpos << result.assembly_error_deg.x() << " deg\n";
    ss << "#   Pitch (绕Y轴): " << std::showpos << result.assembly_error_deg.y() << " deg\n";
    ss << "#   Yaw   (绕Z轴): " << std::showpos << result.assembly_error_deg.z() << " deg\n";
    ss << std::noshowpos;
    ss << "#\n";

    // Legacy output for compatibility
    ss << "# ============================================================\n";
    ss << "# 原始标定数据 (Raw Calibration Data)\n";
    ss << "# camera_optical_frame -> gimbal_link\n";
    ss << "# ============================================================\n";
    ss << std::setprecision(2);
    ss << "# 相机同理想情况的偏角: yaw " << result.misalignment_ypr_deg[0] << " pitch "
       << result.misalignment_ypr_deg[1] << " roll " << result.misalignment_ypr_deg[2]
       << " degree\n";

    ss << std::setprecision(12);
    ss << "\n# 以下为YAML格式数据:\n";
    ss << "urdf_camera_xyz: [" << result.urdf_xyz.x() << ", " << result.urdf_xyz.y() << ", "
       << result.urdf_xyz.z() << "]\n";
    ss << "urdf_camera_rpy: [" << result.urdf_rpy.x() << ", " << result.urdf_rpy.y() << ", "
       << result.urdf_rpy.z() << "]\n";
    ss << "R_camera2gimbal: " << format_flow(mat_to_vector_rowmajor(result.R_camera2gimbal))
       << "\n";
    ss << "t_camera2gimbal: " << format_flow(mat_to_vector_rowmajor(result.t_camera2gimbal))
       << "\n";

    return ss.str();
}

} // namespace rm_calibration
