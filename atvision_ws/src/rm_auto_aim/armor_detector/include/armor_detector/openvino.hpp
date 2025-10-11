#pragma once
#include <array>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

namespace rm_auto_aim {

// 与上文一致的中间结果结构
struct ArmorDet {
    int class_id = -1;              // 0: red, 1: blue
    float score  = 0.f;
    std::array<cv::Point2f, 4> kps; // TL, BL, BR, TR
    cv::Rect box;                   // 原图坐标系
};

class OpenVinoInference {
public:
    struct Result {
        std::vector<ArmorDet> detections;
    };

    OpenVinoInference(
        const std::string& xml, const std::string& device_name = "GPU",
        float score_threshold = 0.15f, float nms_iou = 0.45f, float roi_expand = 1.20f,
        float roi_offset = 2.0f);

    Result infer(const cv::Mat& frame);

    static void sortPoints(std::array<cv::Point2f, 4>& pts);

    static void
        letterboxTopleft(const cv::Mat& src, int net_w, int net_h, cv::Mat& canvas, float& scale);

private:
    ov::Core core;
    std::shared_ptr<ov::Model> model;
    ov::CompiledModel compiled_model;
    ov::InferRequest infer_request;
    ov::Output<const ov::Node> input;
    ov::Output<const ov::Node> output;

    float score_threshold_;
    float nms_iou_;
    float roi_expand_;
    float roi_offset_;
};

} // namespace rm_auto_aim
