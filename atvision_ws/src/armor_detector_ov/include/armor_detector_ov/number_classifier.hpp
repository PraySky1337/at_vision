// number_classifier.hpp
// Number classifier for traditional detection backend
// Migrated from armor_detector package

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include <mutex>

#include "armor_detector_ov/armor_types.hpp"

namespace rm_auto_aim {

// Internal structure for traditional backend classification
struct ClassifiableArmor {
    Light left_light;
    Light right_light;
    ArmorType type = ArmorType::SMALL;
    cv::Mat number_img;
    std::string number;
    float confidence = 0.f;
    std::string classification_result;

    ClassifiableArmor() = default;
    ClassifiableArmor(const Light& l, const Light& r) : left_light(l), right_light(r) {
        if (l.center.x > r.center.x) {
            std::swap(left_light, right_light);
        }
    }

    // Convert to ArmorObject
    ArmorObject toArmorObject() const {
        ArmorObject obj(left_light, right_light);
        obj.cls = armor_string_to_cls(number);
        obj.prob = confidence;
        return obj;
    }
};

class NumberClassifier {
public:
    NumberClassifier(
        const std::string& model_path,
        const std::string& label_path,
        double threshold,
        const std::vector<std::string>& ignore_classes,
        cv::Size input_size = cv::Size(28, 28),
        bool use_softmax = false
    );

    // Extract number ROI from image
    cv::Mat extractNumber(const cv::Mat& src, const ClassifiableArmor& armor) const noexcept;

    // Classify single armor
    void classify(const cv::Mat& src, ClassifiableArmor& armor) noexcept;

    // Filter out ignored classes
    void eraseIgnoreClasses(std::vector<ClassifiableArmor>& armors) noexcept;

    double threshold;

private:
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    std::vector<std::string> ignore_classes_;
    cv::Size input_size_;
    bool use_softmax_;
    mutable std::mutex mutex_;
};

}  // namespace rm_auto_aim
