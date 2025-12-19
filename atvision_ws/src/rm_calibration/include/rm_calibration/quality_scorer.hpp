#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace rm_calibration
{

class QualityScorer
{
public:
  double score(const std::vector<cv::Point2f> & centers_2d, const cv::Size & image_size) const;
};

}  // namespace rm_calibration

