#pragma once
#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>
#include <mutex>

namespace camera_imu_calibration
{

class IntrinsicCalibrator
{
public:
  IntrinsicCalibrator(
    int board_width, 
    int board_height, 
    double square_size,
    int target_samples = 20);

  bool detectCorners(const cv::Mat& image, std::vector<cv::Point2f>& corners, cv::Mat& display_image) const;

  void configureCollection(int target_samples, double quality_threshold);
  bool tryAddSample(const std::vector<cv::Point2f>& corners, const cv::Size& image_size, double score);

  bool calibrate(cv::Mat& camera_matrix, cv::Mat& dist_coeffs, double& rms_error);

  int getCollectedFrames() const;
  int getRequiredFrames() const { return target_samples_; }
  double getAverageQuality() const;

  void reset();

private:
  struct ScoredFrame
  {
    double score{0.0};
    std::vector<cv::Point2f> corners;
    cv::Size image_size;
  };

  struct FrameCompare
  {
    bool operator()(const ScoredFrame& a, const ScoredFrame& b) const
    {
      return a.score > b.score;  // min-heap
    }
  };

  // Checkerboard parameters
  int board_width_;
  int board_height_;
  double square_size_;
  int target_samples_;
  double quality_threshold_;

  std::priority_queue<ScoredFrame, std::vector<ScoredFrame>, FrameCompare> best_frames_;
  mutable std::mutex data_mutex_;

  // Helper methods
  std::vector<cv::Point3f> createObjectPoints() const;
  std::vector<ScoredFrame> getSamplesSnapshot() const;
};

} // namespace camera_imu_calibration
