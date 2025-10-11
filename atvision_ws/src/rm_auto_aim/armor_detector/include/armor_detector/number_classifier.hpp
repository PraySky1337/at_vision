#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include <mutex>

#include "armor_detector/types.hpp"

namespace fyt::auto_aim {

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

    // 提取数字 ROI
    cv::Mat extractNumber(const cv::Mat& src, const Armor& armor) const noexcept;

    // 分类（单个装甲板）
    void classify(const cv::Mat& src, Armor& armor) noexcept;

    // 批量过滤
    void eraseIgnoreClasses(std::vector<Armor>& armors) noexcept;
    double threshold;

private:
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    std::vector<std::string> ignore_classes_;
    cv::Size input_size_;
    bool use_softmax_;                // ⭐ softmax 开关
    mutable std::mutex mutex_;
};

}  // namespace fyt::auto_aim
