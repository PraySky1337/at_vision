// traditional_backend.hpp
// Traditional vision detection backend using light bar matching
// Migrated from armor_detector package

#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <memory>

#include "armor_detector_ov/detector_backend.hpp"
#include "armor_detector_ov/armor_types.hpp"
#include "armor_detector_ov/number_classifier.hpp"

namespace rm_auto_aim {

class TraditionalBackend : public DetectorBackend {
public:
    struct LightParams {
        double min_ratio = 0.1;           // width / height min
        double max_ratio = 0.55;          // width / height max
        double max_angle = 45.0;          // vertical angle
        int color_diff_thresh = 50;       // R-B color difference threshold
    };

    struct ArmorParams {
        double min_light_ratio = 0.7;     // min ratio of light lengths
        double min_small_center_distance = 0.8;
        double max_small_center_distance = 3.2;
        double min_large_center_distance = 3.2;
        double max_large_center_distance = 5.5;
        double max_angle = 35.0;          // horizontal angle
    };

    TraditionalBackend(int binary_thres, const LightParams& l, const ArmorParams& a);

    // DetectorBackend interface
    // Note: roi parameter is ignored - traditional backend always processes full image
    std::vector<ArmorObject> detect(const cv::Mat& image,
                                    const cv::Rect& roi = {}) override;
    void setDetectColor(int color) override { detect_color_ = color; }
    int getDetectColor() const override { return detect_color_; }
    cv::Mat getDebugImage() const override { return debug_img_; }
    cv::Mat getBinaryImage() const override { return binary_img_; }
    bool isInitialized() const override { return initialized_; }
    std::string getBackendName() const override { return "traditional"; }

    // Traditional backend does not support ROI or async
    bool supportsROI() const override { return false; }
    bool supportsAsync() const override { return false; }

    // Initialize classifier (optional, for number recognition)
    void initClassifier(
        const std::string& model_path,
        const std::string& label_path,
        double threshold = 0.7,
        const std::vector<std::string>& ignore_classes = {"negative"},
        bool use_softmax = false);

    // Draw results on image
    void drawResults(cv::Mat& img) const;

    // Get all number images for debug
    cv::Mat getAllNumbersImage() const;

    // Parameters (public for dynamic reconfigure)
    int binary_thres_;
    LightParams light_params_;
    ArmorParams armor_params_;

private:
    cv::Mat preprocessImage(const cv::Mat& rgb_img);
    std::vector<Light> findLights(const cv::Mat& rgb_img, const cv::Mat& bin_img);
    std::vector<ClassifiableArmor> matchLights(const std::vector<Light>& lights);

    bool isLight(const Light& light) const;
    static bool containLight(int i, int j, const std::vector<Light>& lights);
    ArmorType isArmor(const Light& light_1, const Light& light_2) const;

    int detect_color_ = 1;  // 0=BLUE, 1=RED
    bool initialized_ = true;

    std::unique_ptr<NumberClassifier> classifier_;

    // Cached results for debug
    cv::Mat gray_img_;
    cv::Mat binary_img_;
    cv::Mat debug_img_;
    std::vector<Light> lights_;
    std::vector<ClassifiableArmor> armors_;
};

}  // namespace rm_auto_aim
