#ifndef RUNE_DETECTOR_TYPES_HPP_
#define RUNE_DETECTOR_TYPES_HPP_

// 3rd party
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace fyt::rune {

enum class Status {
    SUCCESS,
    ARROW_FAILURE,
    ARMOR_FAILURE,
    CENTER_FAILURE
}; // 成功，箭头检测失败，装甲板检测失败，中心R检测失败

enum class EnemyColor {
    RED   = 0,
    BLUE  = 1,
    WHITE = 2,
};

constexpr double MIN_ROUNDNESS                 = 0.6;            // 最小允许圆度
constexpr double MAX_ROUNDNESS                 = 1.0;            // 最大允许圆度
constexpr int MIN_ARROW_LIGHT_NUM              = 6;              // 流水灯小灯条最小数量
constexpr int MAX_ARROW_LIGHT_NUM              = 15;             // 流水灯小灯条最大数量
constexpr int MIN_ARROW_ASPECT_RATIO           = 2;              // 流水灯最小比例
constexpr int MAX_ARROW_ASPECT_RATIO           = 12;             // 流水灯最大比例
constexpr int MAX_ARROW_AREA                   = 5000;           // 流水灯最大面积
constexpr double GLOBAL_ROI_LENGTH_RATIO       = 1.5;            // 全局裁剪能量机关比例
constexpr double LOCAL_ROI_DISTANCE_RATIO      = 6.6;            // 局部裁剪能量机关比例
constexpr int LOCAL_ROI_WIDTH                  = 200;            // 局部裁剪宽度
constexpr double MAX_CENTER_ASPECT_RATIO       = 3;              // R中心最大比例
constexpr int MIN_TARGET_LIGHT_AREA            = 400;            // 靶最小旋转矩形的最小面积
constexpr int MAX_TARGET_LIGHT_AREA            = 30000;          // 靶最小旋转矩形的最大面积
constexpr int MIN_TARGET_LIGHT_CONTOUR_AREA    = 300;            // 靶轮廓最小面积
constexpr int MAX_TARGET_LIGHT_CONTOUR_AREA    = 10000;          // 靶轮廓最大面积
constexpr double MIN_TARGET_LIGHT_ASPECT_RATIO = 0.5;            // 靶最小比例
constexpr double MAX_TARGET_LIGHT_ASPECT_RATIO = 1.5;            // 靶最大比例
constexpr double ARROW_WIDTH                   = 60;             // 流水灯的宽度
constexpr int MIN_ARROW_LIGHT_AREA             = 20;             // 流水灯小灯条的最小面积
constexpr int MAX_ARROW_LIGHT_AREA             = 250;            // 流水灯小灯条的最大面积
constexpr double MAX_ARROW_LIGHT_ASPECT_RATIO  = 5;              // 流水灯小灯条最大比例
constexpr double MAX_SAME_ARROW_AREA_RATIO     = 5;              // 最大箭头面积相似比例

constexpr double RUNE_PAN_REAL_DIS = 0.15;
constexpr double RUNE_R2PANCENTER  = 0.7;

struct Points {
    cv::Point2f center;
    std::vector<cv::Point2f> corners;
};

struct KeyPoint {
    cv::Point2f lu;                                              // 左上
    cv::Point2f ru;                                              // 右上
    cv::Point2f rd;                                              // 右下
    cv::Point2f ld;                                              // 左下
};

struct Light {
    Light() = default;
    Light(
        const std::vector<cv::Point>& cnt, const cv::Rect2f& globalRoi, const cv::Rect2f& localRoi);

    double roundness;
    std::vector<cv::Point> contour;
    cv::Point2f center;
    cv::Size2f size;
    float angle;
    double contourArea;
    double area;
    cv::RotatedRect rotated;
    double ratio;
};

struct CenterR {
    CenterR() = default;
    void set(const Light& light);
    Light light;
    cv::Point2f center;
    cv::Rect boundingRect;
};

struct Target {
    cv::Point2f center;
    double roll=0;                                       // R中心
    std::vector<cv::Point> contour;
    void set(const Light& l);
    KeyPoint keypnt;

    std::vector<cv::Point3f> points3d = {
        {0.0f,                      0.0f,                                        0.0f}, // P0
        {0.0f,  RUNE_PAN_REAL_DIS / 2.0f, RUNE_R2PANCENTER + RUNE_PAN_REAL_DIS / 2.0f}, // P1
        {0.0f,  RUNE_PAN_REAL_DIS / 2.0f, RUNE_R2PANCENTER - RUNE_PAN_REAL_DIS / 2.0f}, // P2
        {0.0f, -RUNE_PAN_REAL_DIS / 2.0f, RUNE_R2PANCENTER - RUNE_PAN_REAL_DIS / 2.0f}, // P3
        {0.0f, -RUNE_PAN_REAL_DIS / 2.0f, RUNE_R2PANCENTER + RUNE_PAN_REAL_DIS / 2.0f}, // P4
        {0.0f,                      0.0f,                            RUNE_R2PANCENTER}  // P5
    };
};

struct Arrow {
    Arrow() = default;
    void set(const std::vector<Light>& points, const cv::Point2f& roi);
    std::vector<cv::Point> contour;
    cv::Point2f center;
    cv::Size2f size;
    double angle;
    double area;
    cv::RotatedRect rotated;
    double ratio;
    double fillratio;
};

} // namespace fyt::rune
#endif // RUNE_DETECTOR_TYPES_HPP_
