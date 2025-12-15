#include "rm_calibration/intrinsic_calibrator.hpp"

#include <algorithm>
#include <iostream>

namespace camera_imu_calibration {

IntrinsicCalibrator::IntrinsicCalibrator(
    int board_width, int board_height, double square_size, int target_samples)
    : board_width_(board_width)
    , board_height_(board_height)
    , square_size_(square_size)
    , target_samples_(target_samples)
    , quality_threshold_(0.0) {}

bool IntrinsicCalibrator::detectCorners(
    const cv::Mat& image, std::vector<cv::Point2f>& corners, cv::Mat& display_image) const {
    cv::Size board_size(board_width_, board_height_);

    bool found = cv::findChessboardCorners(
        image, board_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if (found) {
        // Refine corner positions
        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image;
        }

        cv::cornerSubPix(
            gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

        const bool aliases_input = !display_image.empty() && display_image.data == image.data
                                && display_image.rows == image.rows
                                && display_image.cols == image.cols
                                && display_image.type() == image.type();
        if (display_image.empty() || aliases_input) {
            // Avoid drawing directly on the input image (callers may score using the original
            // pixels).
            display_image = image.clone();
        } else {
            image.copyTo(display_image);
        }

        cv::drawChessboardCorners(display_image, board_size, corners, found);
    } else if (display_image.empty()) {
        display_image = image;
    }

    return found;
}

void IntrinsicCalibrator::configureCollection(int target_samples, double quality_threshold) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_samples_    = target_samples;
    quality_threshold_ = quality_threshold;
}

bool IntrinsicCalibrator::tryAddSample(
    const std::vector<cv::Point2f>& corners, const cv::Size& image_size, double score) {
    if (score < quality_threshold_ && !best_frames_.empty()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    ScoredFrame frame;
    frame.score      = score;
    frame.corners    = corners;
    frame.image_size = image_size;

    if (best_frames_.size() < static_cast<size_t>(target_samples_)) {
        best_frames_.push(frame);
        return true;
    }

    if (!best_frames_.empty() && score > best_frames_.top().score) {
        best_frames_.pop();
        best_frames_.push(frame);
        return true;
    }

    return false;
}

bool IntrinsicCalibrator::calibrate(
    cv::Mat& camera_matrix, cv::Mat& dist_coeffs, double& rms_error) {
    const auto samples = getSamplesSnapshot();
    if (samples.size() < 3) {
        std::cerr << "Not enough frames collected. Need at least 3, have " << samples.size()
                  << std::endl;
        return false;
    }

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;

    for (const auto& frame : samples) {
        object_points.push_back(createObjectPoints());
        image_points.push_back(frame.corners);
    }

    cv::Size image_size = samples.front().image_size;

    std::vector<cv::Mat> rvecs, tvecs;
    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs   = cv::Mat::zeros(5, 1, CV_64F);

    rms_error = cv::calibrateCamera(
        object_points, image_points, image_size, camera_matrix, dist_coeffs, rvecs, tvecs,
        cv::CALIB_FIX_K3);

    return true;
}

std::vector<cv::Point3f> IntrinsicCalibrator::createObjectPoints() const {
    std::vector<cv::Point3f> corners;
    for (int i = 0; i < board_height_; i++) {
        for (int j = 0; j < board_width_; j++) {
            corners.push_back(cv::Point3f(j * square_size_, i * square_size_, 0.0f));
        }
    }
    return corners;
}

std::vector<IntrinsicCalibrator::ScoredFrame> IntrinsicCalibrator::getSamplesSnapshot() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto queue_copy = best_frames_;
    std::vector<ScoredFrame> frames;
    while (!queue_copy.empty()) {
        frames.push_back(queue_copy.top());
        queue_copy.pop();
    }

    std::sort(frames.begin(), frames.end(), [](const ScoredFrame& a, const ScoredFrame& b) {
        return a.score > b.score;
    });
    return frames;
}

int IntrinsicCalibrator::getCollectedFrames() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return static_cast<int>(best_frames_.size());
}

double IntrinsicCalibrator::getAverageQuality() const {
    const auto frames = getSamplesSnapshot();
    if (frames.empty()) {
        return 0.0;
    }

    double sum = 0.0;
    for (const auto& f : frames) {
        sum += f.score;
    }
    return sum / static_cast<double>(frames.size());
}

void IntrinsicCalibrator::reset() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    while (!best_frames_.empty()) {
        best_frames_.pop();
    }
}

} // namespace camera_imu_calibration
