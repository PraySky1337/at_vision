// Copyright 2022 Chen Jun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"

namespace rm_auto_aim
{
NumberClassifier::NumberClassifier(
  const std::string & model_path, const std::string & label_path, const double thre,
  const std::vector<std::string> & ignore_classes)
: threshold(thre), ignore_classes_(ignore_classes)
{
  net_ = cv::dnn::readNetFromONNX(model_path);

  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
}

void NumberClassifier::extractNumbers(const cv::Mat & src_bgr, std::vector<Armor> & armors)
{
  static const int K_LIGHT_LEN = 12;
  static const int K_WARP_H = 28;
  static const int K_SMALL_W = 32;
  static const int K_LARGE_W = 54;
  static const cv::Size K_ROI_SIZE(20, 28);

  /* ① 仅保留灰度通道 → 后面少一次色彩往返 */
  cv::Mat src_gray;
  cv::cvtColor(src_bgr, src_gray, cv::COLOR_BGR2GRAY);

  for (auto & armor : armors) {
    /* ---- 透视变换 ---- */
    cv::Point2f src_quad[4] = {
      armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
      armor.right_light.bottom};

    const int top_y = (K_WARP_H - K_LIGHT_LEN) / 2 - 1;
    const int bottom_y = top_y + K_LIGHT_LEN;
    const int warp_w = (armor.type == ArmorType::SMALL) ? K_SMALL_W : K_LARGE_W;
    cv::Point2f dst_quad[4] = {
      {0, bottom_y}, {0, top_y}, {warp_w - 1.0f, top_y}, {warp_w - 1.0f, bottom_y}};

    cv::Mat warp_gray;
    cv::warpPerspective(
      src_gray, warp_gray, cv::getPerspectiveTransform(src_quad, dst_quad), {warp_w, K_WARP_H},
      cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    /* ---- 提取数字 ROI（20×28 中央区域） ---- */
    warp_gray =
      warp_gray(cv::Rect((warp_w - K_ROI_SIZE.width) / 2, 0, K_ROI_SIZE.width, K_ROI_SIZE.height));

    /* ---- CLAHE + Otsu 二值化 ---- */
    cv::threshold(warp_gray, warp_gray, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    armor.number_img = warp_gray;  // 直接存灰度二值图
  }
}

void NumberClassifier::classify(std::vector<Armor> & armors)
{
  for (auto & armor : armors) {
    cv::Mat image = armor.number_img.clone();

    // Normalize
    image = image / 255.0;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // Set the input blob for the neural network
    net_.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward();

    // Do softmax
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armor.confidence = confidence;
    armor.number = class_names_[label_id];

    std::stringstream result_ss;
    result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%";
    armor.classfication_result = result_ss.str();
  }

  armors.erase(
    std::remove_if(
      armors.begin(), armors.end(),
      [this](const Armor & armor) {
        if (armor.confidence < threshold) {
          return true;
        }

        for (const auto & ignore_class : ignore_classes_) {
          if (armor.number == ignore_class) {
            return true;
          }
        }

        bool mismatch_armor_type = false;
        if (armor.type == ArmorType::LARGE) {
          mismatch_armor_type =
            armor.number == "outpost" || armor.number == "2" || armor.number == "guard";
        } else if (armor.type == ArmorType::SMALL) {
          mismatch_armor_type = armor.number == "1" || armor.number == "base";
        }
        return mismatch_armor_type;
      }),
    armors.end());
}

}  // namespace rm_auto_aim
