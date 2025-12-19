#include "rm_calibration/intrinsic_calibrator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cfloat>

namespace rm_calibration {

IntrinsicCalibrator::IntrinsicCalibrator(cv::Size pattern_size, double center_distance_m)
    : pattern_size_(pattern_size)
    , center_distance_m_(center_distance_m) {}

std::vector<cv::Point3f> IntrinsicCalibrator::make_object_points() const {
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

bool IntrinsicCalibrator::detect(const cv::Mat& image, std::vector<cv::Point2f>& centers_2d) const {
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

IntrinsicCalibrationResult IntrinsicCalibrator::calibrate(
    const std::vector<std::filesystem::path>& image_paths, bool fix_k3) const {
    IntrinsicCalibrationResult out;

    cv::Size img_size;
    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<std::vector<cv::Point2f>> img_points;

    const auto obj_template = make_object_points();

    for (const auto& p : image_paths) {
        const cv::Mat img = cv::imread(p.string(), cv::IMREAD_COLOR);
        if (img.empty()) {
            continue;
        }

        if (img_size.empty()) {
            img_size = img.size();
        }

        std::vector<cv::Point2f> centers_2d;
        const bool success = detect(img, centers_2d);
        if (!success) {
            continue;
        }

        img_points.emplace_back(std::move(centers_2d));
        obj_points.emplace_back(obj_template);
    }

    if (img_points.size() < 3U || obj_points.size() < 3U || img_size.empty()) {
        out.used_samples = img_points.size();
        return out;
    }

    const int flags = fix_k3 ? cv::CALIB_FIX_K3 : 0;
    const auto criteria =
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);

    std::vector<cv::Mat> rvecs, tvecs;
    cv::calibrateCamera(
        obj_points, img_points, img_size, out.camera_matrix, out.dist_coeffs, rvecs, tvecs, flags,
        criteria);

    double error_sum         = 0.0;
    std::size_t total_points = 0;
    for (std::size_t i = 0; i < obj_points.size(); i++) {
        std::vector<cv::Point2f> reprojected;
        cv::projectPoints(
            obj_points[i], rvecs[i], tvecs[i], out.camera_matrix, out.dist_coeffs, reprojected);
        total_points += reprojected.size();
        for (std::size_t j = 0; j < reprojected.size(); j++) {
            error_sum += cv::norm(img_points[i][j] - reprojected[j]);
        }
    }

    out.used_samples = img_points.size();
    if (total_points > 0U) {
        out.reprojection_error_px = error_sum / static_cast<double>(total_points);
    }

    return out;
}

} // namespace rm_calibration
