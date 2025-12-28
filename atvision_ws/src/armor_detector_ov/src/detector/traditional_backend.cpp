// traditional_backend.cpp
// Traditional vision detection backend using light bar matching
// Migrated from armor_detector package

#include "armor_detector_ov/traditional_backend.hpp"

#include <algorithm>
#include <cmath>
#include <execution>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace rm_auto_aim {

TraditionalBackend::TraditionalBackend(
    int binary_thres, const LightParams& l, const ArmorParams& a)
    : binary_thres_(binary_thres)
    , light_params_(l)
    , armor_params_(a) {}

void TraditionalBackend::initClassifier(
    const std::string& model_path,
    const std::string& label_path,
    double threshold,
    const std::vector<std::string>& ignore_classes,
    bool use_softmax) {
    classifier_ = std::make_unique<NumberClassifier>(
        model_path, label_path, threshold, ignore_classes, cv::Size(28, 28), use_softmax);
}

std::vector<ArmorObject> TraditionalBackend::detect(const cv::Mat& input, const cv::Rect& /*roi*/) {
    // Note: roi parameter is ignored - traditional backend always processes full image
    // 1. Preprocess the image
    binary_img_ = preprocessImage(input);

    // 2. Find lights
    lights_ = findLights(input, binary_img_);

    // 3. Match lights to armors
    armors_ = matchLights(lights_);

    if (!armors_.empty() && classifier_ != nullptr) {
        // Parallel processing for number classification
        std::for_each(
            std::execution::par, armors_.begin(), armors_.end(),
            [this, &input](ClassifiableArmor& armor) {
                // 4. Extract the number image
                armor.number_img = classifier_->extractNumber(input, armor);
                // 5. Do classification
                classifier_->classify(input, armor);
            });

        // 6. Erase the armors with ignore classes
        classifier_->eraseIgnoreClasses(armors_);
    }

    // Convert to ArmorObject
    std::vector<ArmorObject> result;
    result.reserve(armors_.size());
    for (const auto& armor : armors_) {
        result.push_back(armor.toArmorObject());
    }

    // Store debug image
    debug_img_ = input.clone();
    drawResults(debug_img_);

    return result;
}

cv::Mat TraditionalBackend::preprocessImage(const cv::Mat& rgb_img) {
    cv::cvtColor(rgb_img, gray_img_, cv::COLOR_RGB2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img_, binary_img, binary_thres_, 255, cv::THRESH_BINARY);

    return binary_img;
}

std::vector<Light> TraditionalBackend::findLights(
    const cv::Mat& rgb_img, const cv::Mat& bin_img) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<Light> lights;

    for (const auto& contour : contours) {
        if (contour.size() < 6)
            continue;

        auto light = Light(contour);

        if (isLight(light)) {
            int sum_r = 0, sum_b = 0;
            for (const auto& point : contour) {
                sum_r += rgb_img.at<cv::Vec3b>(point.y, point.x)[0];
                sum_b += rgb_img.at<cv::Vec3b>(point.y, point.x)[2];
            }
            if (std::abs(sum_r - sum_b) / static_cast<int>(contour.size())
                > light_params_.color_diff_thresh) {
                light.color = sum_r > sum_b ? EnemyColor::RED : EnemyColor::BLUE;
            }
            lights.emplace_back(light);
        }
    }

    // Sort by x coordinate
    std::sort(lights.begin(), lights.end(), [](const Light& l1, const Light& l2) {
        return l1.center.x < l2.center.x;
    });

    return lights;
}

bool TraditionalBackend::isLight(const Light& light) const {
    // The ratio of light (short side / long side)
    float ratio = static_cast<float>(light.width) / static_cast<float>(light.length);
    bool ratio_ok = light_params_.min_ratio < ratio && ratio < light_params_.max_ratio;

    bool angle_ok = light.tilt_angle < light_params_.max_angle;

    return ratio_ok && angle_ok;
}

std::vector<ClassifiableArmor> TraditionalBackend::matchLights(
    const std::vector<Light>& lights) {
    std::vector<ClassifiableArmor> armors;

    // Convert detect_color_ (int) to EnemyColor for comparison
    EnemyColor target_color = (detect_color_ == 0) ? EnemyColor::BLUE : EnemyColor::RED;

    // Loop all the pairing of lights
    for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
        if (light_1->color != target_color && light_1->color != EnemyColor::WHITE)
            continue;

        double max_iter_width = light_1->length * armor_params_.max_large_center_distance;

        for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
            if (light_2->color != target_color && light_2->color != EnemyColor::WHITE)
                continue;

            if (containLight(
                    static_cast<int>(light_1 - lights.begin()),
                    static_cast<int>(light_2 - lights.begin()), lights)) {
                continue;
            }

            if (light_2->center.x - light_1->center.x > max_iter_width)
                break;

            auto type = isArmor(*light_1, *light_2);
            if (type != ArmorType::INVALID) {
                ClassifiableArmor armor(*light_1, *light_2);
                armor.type = type;
                armors.emplace_back(armor);
            }
        }
    }

    return armors;
}

bool TraditionalBackend::containLight(
    const int i, const int j, const std::vector<Light>& lights) {
    const Light& light_1 = lights.at(i);
    const Light& light_2 = lights.at(j);

    auto points = std::vector<cv::Point2f>{
        light_1.top, light_1.bottom, light_2.top, light_2.bottom};
    auto bounding_rect = cv::boundingRect(points);

    double avg_length = (light_1.length + light_2.length) / 2.0;
    double avg_width = (light_1.width + light_2.width) / 2.0;

    // Only check lights in between
    for (int k = i + 1; k < j; k++) {
        const Light& test_light = lights.at(k);

        // Avoid number interference
        if (test_light.width > 2 * avg_width) {
            continue;
        }
        // Avoid red dot or bullet interference
        if (test_light.length < 0.5 * avg_length) {
            continue;
        }

        if (bounding_rect.contains(test_light.top) ||
            bounding_rect.contains(test_light.bottom) ||
            bounding_rect.contains(test_light.center)) {
            return true;
        }
    }
    return false;
}

ArmorType TraditionalBackend::isArmor(const Light& light_1, const Light& light_2) const {
    // Ratio of the length of 2 lights (short side / long side)
    float light_length_ratio = light_1.length < light_2.length
                                 ? static_cast<float>(light_1.length / light_2.length)
                                 : static_cast<float>(light_2.length / light_1.length);
    bool light_ratio_ok = light_length_ratio > armor_params_.min_light_ratio;

    // Distance between the center of 2 lights (unit: light length)
    float avg_light_length = static_cast<float>((light_1.length + light_2.length) / 2);
    float center_distance =
        static_cast<float>(cv::norm(light_1.center - light_2.center) / avg_light_length);
    bool center_distance_ok =
        (armor_params_.min_small_center_distance <= center_distance &&
         center_distance < armor_params_.max_small_center_distance) ||
        (armor_params_.min_large_center_distance <= center_distance &&
         center_distance < armor_params_.max_large_center_distance);

    // Angle of light center connection
    cv::Point2f diff = light_1.center - light_2.center;
    float angle = static_cast<float>(std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180);
    bool angle_ok = angle < armor_params_.max_angle;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

    // Judge armor type
    if (is_armor) {
        return center_distance > armor_params_.min_large_center_distance
                   ? ArmorType::LARGE
                   : ArmorType::SMALL;
    }
    return ArmorType::INVALID;
}

void TraditionalBackend::drawResults(cv::Mat& img) const {
    // Draw Lights
    for (const auto& light : lights_) {
        auto line_color =
            light.color == EnemyColor::RED ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 255, 0);
        cv::line(img, light.top, light.bottom, line_color, 1);
    }

    // Draw Armors
    for (const auto& armor : armors_) {
        cv::line(img, armor.left_light.top, armor.right_light.bottom,
                 cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        cv::line(img, armor.left_light.bottom, armor.right_light.top,
                 cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

        cv::Point2f center = (armor.left_light.center + armor.right_light.center) * 0.5f;
        cv::circle(img, center, 2, cv::Scalar(0, 255, 0), 2);

        // Show number and confidence
        std::string text = armorTypeToString(armor.type) + " " + armor.classification_result;
        cv::putText(img, text, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 255), 2);
    }
}

cv::Mat TraditionalBackend::getAllNumbersImage() const {
    if (armors_.empty()) {
        return cv::Mat(cv::Size(20, 28), CV_8UC1);
    }

    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (const auto& armor : armors_) {
        if (!armor.number_img.empty()) {
            number_imgs.emplace_back(armor.number_img);
        }
    }

    if (number_imgs.empty()) {
        return cv::Mat(cv::Size(20, 28), CV_8UC1);
    }

    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
}

}  // namespace rm_auto_aim
