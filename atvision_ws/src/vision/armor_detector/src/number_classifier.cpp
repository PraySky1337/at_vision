#include "armor_detector/number_classifier.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <algorithm>
#include <fmt/format.h>

namespace fyt::auto_aim {

NumberClassifier::NumberClassifier(
    const std::string& model_path,
    const std::string& label_path,
    double threshold,
    const std::vector<std::string>& ignore_classes,
    cv::Size input_size,
    bool use_softmax)
    : threshold(threshold),
      ignore_classes_(ignore_classes),
      input_size_(input_size),
      use_softmax_(use_softmax) {
    net_ = cv::dnn::readNetFromONNX(model_path);

    std::ifstream label_file(label_path);
    std::string line;
    while (std::getline(label_file, line)) {
        class_names_.push_back(line);
    }
}

cv::Mat NumberClassifier::extractNumber(const cv::Mat& src, const Armor& armor) const noexcept {
    // 光条长度
    static const int light_length = 12;
    // 透视变换后的高度
    static const int warp_height = 28;
    static const int small_armor_width = 32;
    static const int large_armor_width = 54;
    // ROI 区域
    static const cv::Size roi_size(20, 28);

    cv::Point2f lights_vertices[4] = {
        armor.left_light.bottom,
        armor.left_light.top,
        armor.right_light.top,
        armor.right_light.bottom
    };

    const int top_light_y    = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width     = (armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width);

    cv::Point2f target_vertices[4] = {
        cv::Point(0, bottom_light_y),
        cv::Point(0, top_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };

    cv::Mat number_image;
    auto M = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, M, cv::Size(warp_width, warp_height));

    // 截取 ROI
    number_image = number_image(cv::Rect(
        cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // 灰度 + 二值化
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // ⭐ 如果当前尺寸和 input_size_ 不符，则 resize
    if (number_image.size() != input_size_) {
        cv::resize(number_image, number_image, input_size_);
    }

    return number_image;
}

void NumberClassifier::classify(const cv::Mat& src, Armor& armor) noexcept {
    cv::Mat input = armor.number_img / 255.0;

    cv::Mat blob;
    cv::dnn::blobFromImage(input, blob);

    mutex_.lock();
    net_.setInput(blob);
    cv::Mat outputs = net_.forward().clone();
    mutex_.unlock();

    double confidence;
    int label_id = -1;

    if (use_softmax_) {
        // ⭐ 旧版网络，需要 softmax
        float max_val = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat exp_scores;
        cv::exp(outputs - max_val, exp_scores);
        float sum = static_cast<float>(cv::sum(exp_scores)[0]);
        exp_scores /= sum;

        cv::Point class_id_point;
        minMaxLoc(exp_scores.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        label_id = class_id_point.x;
    } else {
        // ⭐ 新版网络，直接取最大值
        cv::Point class_id_point;
        minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        label_id = class_id_point.x;
    }

    armor.confidence = confidence;
    armor.number     = class_names_[label_id];
    armor.classfication_result = fmt::format("{}:{:.1f}%", armor.number, armor.confidence * 100.0);
}

void NumberClassifier::eraseIgnoreClasses(std::vector<Armor>& armors) noexcept {
    armors.erase(
        std::remove_if(
            armors.begin(), armors.end(),
            [this](const Armor& armor) {
                if (armor.confidence < threshold) {
                    return true;
                }
                for (const auto& ignore_class : ignore_classes_) {
                    if (armor.number == ignore_class) {
                        return true;
                    }
                }
                bool mismatch = false;
                if (armor.type == ArmorType::LARGE) {
                    mismatch = armor.number == "outpost" || armor.number == "2" || armor.number == "sentry";
                } else if (armor.type == ArmorType::SMALL) {
                    mismatch = armor.number == "1" || armor.number == "base";
                }
                return mismatch;
            }),
        armors.end());
}

}  // namespace fyt::auto_aim
