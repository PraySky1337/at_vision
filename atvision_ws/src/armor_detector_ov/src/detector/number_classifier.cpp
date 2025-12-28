// number_classifier.cpp
// Number classifier for traditional detection backend
// Migrated from armor_detector package

#include "armor_detector_ov/number_classifier.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <algorithm>
#include <utility>

namespace rm_auto_aim {

NumberClassifier::NumberClassifier(
    const std::string& model_path,
    const std::string& label_path,
    double threshold,
    const std::vector<std::string>& ignore_classes,
    cv::Size input_size,
    bool use_softmax)
    : threshold(threshold),
      ignore_classes_(ignore_classes),
      input_size_(std::move(input_size)),
      use_softmax_(use_softmax) {
    net_ = cv::dnn::readNetFromONNX(model_path);

    std::ifstream label_file(label_path);
    std::string line;
    while (std::getline(label_file, line)) {
        class_names_.push_back(line);
    }
}

cv::Mat NumberClassifier::extractNumber(const cv::Mat& src, const ClassifiableArmor& armor) const noexcept {
    // Light length in warped image
    static const int light_length = 12;
    // Warp height
    static const int warp_height = 28;
    static const int small_armor_width = 32;
    static const int large_armor_width = 54;
    // ROI size
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

    // Extract ROI
    number_image = number_image(cv::Rect(
        cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Convert to grayscale and binarize
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Resize if needed
    if (number_image.size() != input_size_) {
        cv::resize(number_image, number_image, input_size_);
    }

    return number_image;
}

void NumberClassifier::classify(const cv::Mat& src, ClassifiableArmor& armor) noexcept {
    cv::Mat input = armor.number_img / 255.0;

    cv::Mat blob;
    cv::dnn::blobFromImage(input, blob);

    std::lock_guard<std::mutex> lock(mutex_);
    net_.setInput(blob);
    cv::Mat outputs = net_.forward().clone();

    double confidence;
    int label_id = -1;

    if (use_softmax_) {
        // Old network: needs softmax
        float max_val = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat exp_scores;
        cv::exp(outputs - max_val, exp_scores);
        float sum = static_cast<float>(cv::sum(exp_scores)[0]);
        exp_scores /= sum;

        cv::Point class_id_point;
        minMaxLoc(exp_scores.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        label_id = class_id_point.x;
    } else {
        // New network: directly take max
        cv::Point class_id_point;
        minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        label_id = class_id_point.x;
    }

    armor.confidence = static_cast<float>(confidence);
    armor.number = class_names_[label_id];

    // Format classification result
    char buf[64];
    snprintf(buf, sizeof(buf), "%s:%.1f%%", armor.number.c_str(), armor.confidence * 100.0);
    armor.classification_result = buf;
}

void NumberClassifier::eraseIgnoreClasses(std::vector<ClassifiableArmor>& armors) noexcept {
    armors.erase(
        std::remove_if(
            armors.begin(), armors.end(),
            [this](const ClassifiableArmor& armor) {
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

}  // namespace rm_auto_aim
