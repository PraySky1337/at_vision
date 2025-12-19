#pragma once

#include <opencv2/core.hpp>

#include <filesystem>
#include <string>
#include <vector>

#include "rm_calibration/types.hpp"

namespace rm_calibration {

struct SampleFile {
    std::filesystem::path image_path;
    std::filesystem::path tf_path;
};

class ExtrinsicCalibrator {
public:
    ExtrinsicCalibrator(cv::Size pattern_size, double center_distance_m);

    bool detect(const cv::Mat& image, std::vector<cv::Point2f>& centers_2d) const;

    static bool load_gripper2base_from_txt(const std::filesystem::path& path, Pose& out_pose);

    ExtrinsicCalibrationResult calibrate(
        const std::vector<SampleFile>& samples, const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs, bool use_gripper_translation) const;

    static std::string format_extrinsic_yaml(
        const ExtrinsicCalibrationResult& result, const std::string& base_frame,
        const std::string& gimbal_frame);

private:
    std::vector<cv::Point3f> make_object_points() const;

    cv::Size pattern_size_;
    double center_distance_m_ = 0.0;
};

} // namespace rm_calibration
