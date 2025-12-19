#pragma once

#include <opencv2/core.hpp>

#include <filesystem>
#include <vector>

#include "rm_calibration/types.hpp"

namespace rm_calibration {

class IntrinsicCalibrator {
public:
    IntrinsicCalibrator(cv::Size pattern_size, double center_distance_m);

    bool detect(const cv::Mat& image, std::vector<cv::Point2f>& centers_2d) const;

    IntrinsicCalibrationResult
        calibrate(const std::vector<std::filesystem::path>& image_paths, bool fix_k3) const;

private:
    std::vector<cv::Point3f> make_object_points() const;

    cv::Size pattern_size_;
    double center_distance_m_ = 0.0;
};

} // namespace rm_calibration
