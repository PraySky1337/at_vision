#include "rm_calibration/quality_scorer.hpp"

namespace rm_calibration
{

double QualityScorer::score(const std::vector<cv::Point2f> & centers_2d, const cv::Size & image_size)
  const
{
  if (centers_2d.empty() || image_size.empty()) {
    return 0.0;
  }
  return 1.0;
}

}  // namespace rm_calibration

