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

    ss << std::setprecision(2);
    ss << "# 相机同理想情况的偏角: yaw" << result.misalignment_ypr_deg[0] << " pitch"
       << result.misalignment_ypr_deg[1] << " roll" << result.misalignment_ypr_deg[2]
       << " degree\n";

    ss << std::setprecision(12);
    ss << "R_camera2gimbal: " << format_flow(mat_to_vector_rowmajor(result.R_camera2gimbal))
       << "\n";
    ss << "t_camera2gimbal: " << format_flow(mat_to_vector_rowmajor(result.t_camera2gimbal))
       << "\n";

    return ss.str();
}

} // namespace rm_calibration
