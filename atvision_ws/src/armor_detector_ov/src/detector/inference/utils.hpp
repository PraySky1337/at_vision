#pragma once

#include "armor_detector_ov/armor_types.hpp"
#include "opencv2/imgproc.hpp"
#include <Eigen/Dense>

namespace rm_auto_aim {

struct GridAndStride {
    int grid0{}, grid1{}, stride{};
};

inline float triArea(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) {
    return std::fabs(0.5f * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)));
}

template <int INPUT_H = 416, int INPUT_W = 416>
cv::Mat scaledResize(const cv::Mat& img, Eigen::Matrix<float, 3, 3>& T) {
    // 与 TUP 原实现保持一致：对称 padding + 线性缩放
    const float r = std::min(
        static_cast<float>(INPUT_W) / static_cast<float>(img.cols),
        static_cast<float>(INPUT_H) / static_cast<float>(img.rows));

    const int unpad_w = static_cast<int>(r * static_cast<float>(img.cols));
    const int unpad_h = static_cast<int>(r * static_cast<float>(img.rows));

    int dw = INPUT_W - unpad_w;
    int dh = INPUT_H - unpad_h;

    dw /= 2;
    dh /= 2;

    // 还原矩阵：将网络坐标 -> 原图坐标
    // 先去掉边，再除缩放
    T << 1.f / r, 0.f, -static_cast<float>(dw) / r, 0.f, 1.f / r, -static_cast<float>(dh) / r, 0.f,
        0.f, 1.f;

    cv::Mat re, out;
    cv::resize(img, re, cv::Size(unpad_w, unpad_h));
    cv::copyMakeBorder(re, out, dh, dh, dw, dw, cv::BORDER_CONSTANT);
    return out;
}

inline void generate_grids_and_stride(
    int W, int H, const std::vector<int>& strides, std::vector<GridAndStride>& gs) {
    gs.clear();
    gs.reserve((W / 8) * (H / 8) + (W / 16) * (H / 16) + (W / 32) * (H / 32));
    for (int s : strides) {
        const int gw = W / s, gh = H / s;
        for (int g1 = 0; g1 < gh; ++g1)
            for (int g0 = 0; g0 < gw; ++g0)
                gs.push_back({g0, g1, s});
    }
}

inline float area4(const cv::Point2f p[4]) {
    return triArea(p[0], p[1], p[2]) + triArea(p[0], p[2], p[3]);
}

inline float iou(const cv::Rect2f& a, const cv::Rect2f& b) {
    float inter = (a & b).area();
    float uni   = a.area() + b.area() - inter;
    return uni > 0 ? inter / uni : 0.f;
}

inline std::vector<ArmorObject> topKAndNms(std::vector<ArmorObject>& v, int topk, float nms) {
    if (v.empty())
        return {};

    std::sort(v.begin(), v.end(), [](const ArmorObject& A, const ArmorObject& B) {
        return A.prob > B.prob;
    });
    if ((int)v.size() > topk)
        v.resize(topk);

    std::vector<char> rm(v.size(), 0);
    std::vector<ArmorObject> keep;
    keep.reserve(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        if (rm[i])
            continue;
        keep.push_back(v[i]);
        for (size_t j = i + 1; j < v.size(); ++j) {
            if (rm[j])
                continue;
            if (iou(v[i].rect, v[j].rect) > nms)
                rm[j] = 1;
        }
    }
    return keep;
}
} // namespace rm_auto_aim