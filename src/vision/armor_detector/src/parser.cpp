/**
 * @projectName armor_auto_aim
 * @file parser.cpp
 * @brief
 *
 * @author yx
 * @date 2023-10-27 19:57
 */

#include "armor_detector/parser.hpp"

#include "armor_detector/inference.hpp"

namespace armor_auto_aim {
cv::Mat scaledResize(const cv::Mat& src, Eigen::Matrix<float, 3, 3>& transformation_matrix) {
    auto width    = static_cast<float>(src.cols);
    auto height   = static_cast<float>(src.rows);
    float rate    = std::min(Inference::INPUT_WIDTH / width, Inference::INPUT_WIDTH / height);
    int updated_w = static_cast<int>(rate * width);
    int updated_h = static_cast<int>(rate * height);
    int dw        = (Inference::INPUT_WIDTH - updated_w) / 2;
    int dh        = (Inference::INPUT_HEIGHT - updated_h) / 2;

    transformation_matrix << 1.0f / rate, 0.0f, -dw / rate, 0.0f, 1.0f / rate, -dh / rate, 0.0f,
        0.0f, 1.0f;
    cv::Mat resized, out;
    cv::resize(src, resized, cv::Size(updated_w, updated_h));
    cv::copyMakeBorder(resized, out, dh, dh, dw, dw, cv::BORDER_CONSTANT);

    return out;
}

void decodeOutputs(
    const float* inference_result_ptr, const int& img_w, const int& img_h,
    const Eigen::Matrix<float, 3, 3>& transform_matrix,
    std::vector<InferenceResult>* inference_armors) {
    if (inference_result_ptr == nullptr)
        return;

    std::vector<InferenceResult> tmp_armors;
    std::vector<GridAndStride> grid_strides;
    std::vector<int> strides = {8, 16, 32};
    // 解析
    generateGridsAndStride(Inference::INPUT_WIDTH, Inference::INPUT_HEIGHT, strides, &grid_strides);
    generateYoloxProposals(
        inference_result_ptr, grid_strides, transform_matrix, k_BoundBoxConfidenceThreshold,
        &tmp_armors);
    // 最多保留的推理出的装甲板数量
    if (tmp_armors.size() >= k_InferenceResultPollMaxVal)
        tmp_armors.resize(k_InferenceResultPollMaxVal);
    // 非极大值抑制
    nonMaximumSuppression(*inference_armors, tmp_armors, k_NmsThreshold);
    // 用多个装甲板的四个顶点的均值作为最终值
    for (auto& armor : *inference_armors) {
        if (armor.points.size() >= 8) {
            auto number = armor.points.size() / 4;
            cv::Point2f averaged_apex[4]; // 均值化后的装甲板四个顶点
            for (size_t i = 0; i < armor.points.size(); ++i)
                averaged_apex[i % 4] += armor.points[i];
            for (auto& item : averaged_apex) {
                item.x = item.x / static_cast<float>(number);
                item.y = item.y / static_cast<float>(number);
            }
            std::copy(
                averaged_apex, averaged_apex + sizeof(averaged_apex) / sizeof(cv::Point2f),
                armor.armor_apex);
        }
        armor.area = static_cast<int>(quadrilateralArea(armor.armor_apex)); // TODO: 是否需要更改
    }
}

void generateGridsAndStride(
    const int& target_w, const int& target_h, const std::vector<int>& strides,
    std::vector<GridAndStride>* grid_strides) {
    for (const auto& stride : strides) {
        int num_grid_w = target_w / stride;
        int num_grid_h = target_h / stride;
        for (int g1 = 0; g1 < num_grid_h; g1++) {
            for (int g0 = 0; g0 < num_grid_w; g0++)
                grid_strides->push_back(GridAndStride{g0, g1, stride});
        }
    }
}

void generateYoloxProposals(
    const float* inference_result_ptr, const std::vector<GridAndStride>& grid_strides,
    const Eigen::Matrix<float, 3, 3>& transform_matrix,
    const float& probability_confidence_threshold, std::vector<InferenceResult>* inference_armors) {
    const int num_anchors = static_cast<int>(grid_strides.size());
    // Travel all the anchors
    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {
        // 通过网格、步长、偏移获取值
        const auto grid0    = static_cast<float>(grid_strides[anchor_idx].grid0);
        const auto grid1    = static_cast<float>(grid_strides[anchor_idx].grid1);
        const auto stride   = static_cast<float>(grid_strides[anchor_idx].stride);
        const int basic_pos = anchor_idx * (9 + k_NumberColor + k_NumberClassification);
        float x_1           = (inference_result_ptr[basic_pos + 0] + grid0) * stride;
        float y_1           = (inference_result_ptr[basic_pos + 1] + grid1) * stride;
        float x_2           = (inference_result_ptr[basic_pos + 2] + grid0) * stride;
        float y_2           = (inference_result_ptr[basic_pos + 3] + grid1) * stride;
        float x_3           = (inference_result_ptr[basic_pos + 4] + grid0) * stride;
        float y_3           = (inference_result_ptr[basic_pos + 5] + grid1) * stride;
        float x_4           = (inference_result_ptr[basic_pos + 6] + grid0) * stride;
        float y_4           = (inference_result_ptr[basic_pos + 7] + grid1) * stride;
        // 获取目标框概率
        auto argMax = [](const float* ptr, const int& len) -> int {
            int max_arg = 0;
            for (int i = 1; i < len; i++) {
                if (ptr[i] > ptr[max_arg])
                    max_arg = i;
            }
            return max_arg;
        };
        int box_color = argMax(inference_result_ptr + basic_pos + 9, k_NumberColor);
        int box_class =
            argMax(inference_result_ptr + basic_pos + 9 + k_NumberColor, k_NumberClassification);
        float box_probability = inference_result_ptr[basic_pos + 8];
        // 筛选并生成装甲板对象
        if (box_probability >= probability_confidence_threshold) {
            InferenceResult armor;
            Eigen::Matrix<float, 3, 4> tmp_apex;               // 顶点
            Eigen::Matrix<float, 3, 4> armor_apex_destination; // 目标顶点
            tmp_apex << x_1, x_2, x_3, x_4, y_1, y_2, y_3, y_4, 1, 1, 1, 1;
            armor_apex_destination = transform_matrix * tmp_apex;
            for (int i = 0; i < 4; i++) {
                armor.armor_apex[i] =
                    cv::Point2f(armor_apex_destination(0, i), armor_apex_destination(1, i));
                armor.points.push_back(armor.armor_apex[i]);
            }
            std::vector<cv::Point2f> tmp(armor.armor_apex, armor.armor_apex + 4);
            armor.rect           = cv::boundingRect(tmp);
            armor.classification = box_class;
            armor.color          = box_color;
            armor.probability    = box_probability;

            inference_armors->push_back(armor);
        }

    } // point anchor loo
}

void nonMaximumSuppression(
    std::vector<InferenceResult>& selected_armors, std::vector<InferenceResult>& inference_armors,
    const float& threshold) {
    /**
     * @brief 计算交并比(iou)
     */
    auto getIou = [](const InferenceResult& a, const InferenceResult& b) -> float {
        float inter_area = (a.rect & b.rect).area();
        float union_area = (a.rect | b.rect).area();
        return inter_area / union_area;
    };

    std::sort(
        inference_armors.begin(), inference_armors.end(),
        [](const InferenceResult& a, const InferenceResult& b) {
            return a.probability > b.probability; // 降序
        });
    while (!inference_armors.empty()) {
        InferenceResult& selected_armor = inference_armors[0];
        selected_armors.push_back(selected_armor);
        inference_armors.erase(inference_armors.begin());
        // 移除重叠区域大的边界框(en: Remove overlapping bounding boxes with large overlapping
        // areas)
        auto new_end = std::remove_if(
            inference_armors.begin(), inference_armors.end(),
            [&selected_armor, threshold, getIou](InferenceResult& item) -> bool {
                float iou = getIou(selected_armor, item);
                if (iou > k_UseToMeanIouThreshold
                    && selected_armor.classification == item.classification
                    && selected_armor.color == item.color
                    && abs(selected_armor.probability - item.probability)
                           < k_MergeConfidenceError) {
                    for (auto& i : item.armor_apex)
                        selected_armor.points.push_back(std::move(i));
                }
                return iou > threshold;
            });
        inference_armors.erase(new_end, inference_armors.end());
        //        for (int i = static_cast<int>(inference_armors.size()) - 1; i >= 0; --i) {
        //            auto& item = inference_armors[i];
        //            if (selected_armor.classification == item.classification &&
        //                selected_armor.color == item.color &&
        //                abs(selected_armor.probability - item.probability) <
        //                k_MergeConfidenceError && getIou(selected_armor, item) > threshold) {
        //                    inference_armors.erase(inference_armors.begin() + i);
        //            }
        //        }
    }
}

float quadrilateralArea(cv::Point2f points[4]) {
    float x1 = points[0].x, y1 = points[0].y, x2 = points[1].x, y2 = points[1].y, x3 = points[2].x,
          y3 = points[2].y, x4 = points[3].x, y4 = points[3].y;

    return 0.5f * std::abs(x1 * (y2 - y4) + x2 * (y3 - y1) + x3 * (y4 - y2) + x4 * (y1 - y3));
}
} // namespace armor_auto_aim
