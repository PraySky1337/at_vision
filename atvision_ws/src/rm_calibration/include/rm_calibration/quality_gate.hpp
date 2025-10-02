#pragma once

#include <opencv2/opencv.hpp>

namespace rm_calibration {
struct QualityParams {
    // 清晰度
    double min_var_laplacian = 120.0; // 拉普拉斯方差阈值(>=)
    // 亮度与饱和
    double min_mean_gray       = 50.0;  // 平均灰度下限
    double max_mean_gray       = 200.0; // 平均灰度上限
    double max_saturated_ratio = 0.02;  // 过曝像素占比上限(>250或<5)
    // 图案面积与边界
    double min_area_frac = 0.04; // 棋盘/圆点外接框面积占整图下限
    double max_area_frac = 0.45; // 上限
    int border_margin_px = 10;   // 边界保留(像素)
    // 形状健壮性(避免极端瘦长/退化)
    double min_short_side_px = 25.0; // 最短边最小像素
    double min_aspect_ratio  = 0.25; // min(w,h)/max(w,h)下限
};

struct QualityResult {
    bool ok = false;
    std::string reason;              // 失败原因（ok=true时为空）
    double var_lap   = 0.0;          // 指标回传，便于记录
    double mean_gray = 0.0;
    double sat_ratio = 0.0;
    double area_frac = 0.0;
};

// corners: 已检测到的角点/圆心(必须是完整数量)
// pattern: 检测到的列col、行row（只用于面积/形状估计）
// img: 原始图像(BGR/Gray均可)
inline QualityResult qualityGate(
    const cv::Mat& img, const std::vector<cv::Point2f>& corners, const cv::Size& pattern,
    const QualityParams& qp = {}) {
    QualityResult qr;
    // 0) 基本合法性
    if (img.empty()) {
        qr.reason = "empty image";
        return qr;
    }
    if ((int)corners.size() != pattern.width * pattern.height) {
        qr.reason = "incomplete pattern";
        return qr;
    }

    // 1) 灰度/清晰度
    cv::Mat gray;
    if (img.channels() == 3)
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else
        gray = img;

    cv::Scalar mean_s, std_s;
    cv::meanStdDev(gray, mean_s, std_s);
    qr.mean_gray = mean_s[0];

    // 饱和/欠曝比例
    int total    = gray.rows * gray.cols;
    int sat_hi   = cv::countNonZero(gray > 250);
    int sat_lo   = cv::countNonZero(gray < 5);
    qr.sat_ratio = double(sat_hi + sat_lo) / std::max(1, total);
    if (qr.mean_gray < qp.min_mean_gray) {
        qr.reason = "too dark";
        return qr;
    }
    if (qr.mean_gray > qp.max_mean_gray) {
        qr.reason = "too bright";
        return qr;
    }
    if (qr.sat_ratio > qp.max_saturated_ratio) {
        qr.reason = "too many saturated pixels";
        return qr;
    }

    // 清晰度（拉普拉斯方差）
    cv::Mat lap;
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar m, s;
    cv::meanStdDev(lap, m, s);
    qr.var_lap = s[0] * s[0];
    if (qr.var_lap < qp.min_var_laplacian) {
        qr.reason = "blurry (varLap too low)";
        return qr;
    }

    // 2) 面积/边界/形状
    cv::Rect bb  = cv::boundingRect(corners);
    qr.area_frac = (bb.area() * 1.0) / (img.cols * img.rows);
    if (qr.area_frac < qp.min_area_frac) {
        qr.reason = "pattern too small";
        return qr;
    }
    if (qr.area_frac > qp.max_area_frac) {
        qr.reason = "pattern too large";
        return qr;
    }

    // 边界留白
    if (bb.x < qp.border_margin_px || bb.y < qp.border_margin_px
        || bb.x + bb.width > img.cols - qp.border_margin_px
        || bb.y + bb.height > img.rows - qp.border_margin_px) {
        qr.reason = "too close to border";
        return qr;
    }

    // 形状健壮性（避免极度扁/退化）
    cv::RotatedRect rr = cv::minAreaRect(corners);
    double w = rr.size.width, h = rr.size.height;
    double mn = std::min(w, h), mx = std::max(w, h);
    if (mn < qp.min_short_side_px) {
        qr.reason = "pattern too thin";
        return qr;
    }
    if (mn / std::max(1.0, mx) < qp.min_aspect_ratio) {
        qr.reason = "pattern degenerate";
        return qr;
    }

    // 通过
    qr.ok = true;
    qr.reason.clear();
    return qr;
}

} // namespace rm_calibration