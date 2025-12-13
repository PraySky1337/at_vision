#ifndef CAMERA_IMU_CALIBRATION__EXTRINSIC_CALIBRATOR_HPP_
#define CAMERA_IMU_CALIBRATION__EXTRINSIC_CALIBRATOR_HPP_

#include <Eigen/Dense>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>

namespace camera_imu_calibration {

class ExtrinsicCalibrator {
public:
    ExtrinsicCalibrator(
        int board_width, int board_height, double square_size, const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs, int target_samples = 30);

    bool detectSample(
        const cv::Mat& image, const Eigen::Matrix3d& R_imu, std::vector<cv::Point2f>& corners,
        cv::Mat& display_image, cv::Mat& rvec, cv::Mat& tvec, double& reprojection_error) const;

    void configureCollection(int target_samples, double quality_threshold);
    bool tryAddSample(
        const cv::Mat& rvec, const cv::Mat& tvec, const Eigen::Matrix3d& R_imu,
        const Eigen::Vector3d& t_imu, const std::vector<cv::Point2f>& corners, double score,
        double reprojection_error);

    bool calibrate(cv::Mat& R_cam_to_imu, cv::Mat& t_cam_to_imu, double& error);

    int getCollectedSamples() const;
    int getRequiredSamples() const { return target_samples_; }
    double getAverageQuality() const;

    void reset();

private:
    struct CalibrationSample {
        cv::Mat rvec_camera;
        cv::Mat tvec_camera;
        Eigen::Matrix3d R_imu;
        Eigen::Vector3d t_imu{Eigen::Vector3d::Zero()};
        std::vector<cv::Point2f> corners;
        double score{0.0};
        double reprojection_error{0.0};
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct SampleCompare {
        bool operator()(const CalibrationSample& a, const CalibrationSample& b) const {
            return a.score > b.score; // min-heap
        }
    };
    using SampleContainer =
        std::vector<CalibrationSample, Eigen::aligned_allocator<CalibrationSample>>;
    using SampleQueue = std::priority_queue<CalibrationSample, SampleContainer, SampleCompare>;

    // Checkerboard parameters
    int board_width_;
    int board_height_;
    double square_size_;
    int target_samples_;
    double quality_threshold_;

    // Camera intrinsics
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    SampleQueue best_samples_;
    mutable std::mutex data_mutex_;

    // Helper methods
    std::vector<cv::Point3f> createObjectPoints() const;
    Eigen::Matrix3d cvMatToEigen(const cv::Mat& mat) const;
    cv::Mat eigenToCvMat(const Eigen::Matrix3d& mat) const;
    double computeReprojectionError(
        const std::vector<cv::Point3f>& object_points, const std::vector<cv::Point2f>& image_points,
        const cv::Mat& rvec, const cv::Mat& tvec) const;
    SampleContainer getSamplesSnapshot() const;
};

} // namespace camera_imu_calibration

#endif // CAMERA_IMU_CALIBRATION__EXTRINSIC_CALIBRATOR_HPP_
