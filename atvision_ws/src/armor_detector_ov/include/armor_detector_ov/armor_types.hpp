// armor_types.hpp
// Unified armor detection data types
// Copyright (C) 2024

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <numeric>
#include <string>
#include <array>
#include <vector>

namespace rm_auto_aim {

// ============== Constants ==============

// Armor size, Unit: m
constexpr double SMALL_ARMOR_WIDTH  = 135.0 / 1000.0;
constexpr double SMALL_ARMOR_HEIGHT = 55.0 / 1000.0;
constexpr double LARGE_ARMOR_WIDTH  = 230.0 / 1000.0;
constexpr double LARGE_ARMOR_HEIGHT = 55.0 / 1000.0;

// 15 degree in rad
constexpr double FIFTEEN_DEGREE_RAD = 15.0 * CV_PI / 180.0;

// ============== Enums ==============

// Armor type
enum class ArmorType { SMALL, LARGE, INVALID };

inline std::string armorTypeToString(ArmorType type) {
    switch (type) {
    case ArmorType::SMALL: return "small";
    case ArmorType::LARGE: return "large";
    default: return "invalid";
    }
}

// Enemy color
enum class EnemyColor { BLUE = 0, RED = 1, GRAY = 2, PURPLE = 3, WHITE = 4, UNKNOWN = -1 };

// ============== Utility Functions (defined early for use in structs) ==============

inline std::string armor_cls_to_string(int cls) {
    switch (cls) {
    case 0: return "sentry";
    case 1: return "1";
    case 2: return "2";
    case 3: return "3";
    case 4: return "4";
    case 5: return "5";
    case 6: return "outpost";
    case 7: return "base_small";
    case 8: return "base_large";
    default: return "UNKNOWN";
    }
}

inline int armor_string_to_cls(const std::string& str) {
    if (str == "sentry") return 0;
    if (str == "1") return 1;
    if (str == "2") return 2;
    if (str == "3") return 3;
    if (str == "4") return 4;
    if (str == "5") return 5;
    if (str == "outpost") return 6;
    if (str == "base_small" || str == "base") return 7;
    if (str == "base_large") return 8;
    return -1;
}

inline std::string armor_color_to_string(int color) {
    switch (color) {
    case 0: return "B"; // blue
    case 1: return "R"; // red
    case 2: return "G"; // gray
    case 3: return "P"; // purple
    default: return "UNKNOWN";
    }
}

inline std::string enemyColorToString(EnemyColor color) {
    switch (color) {
    case EnemyColor::BLUE: return "BLUE";
    case EnemyColor::RED: return "RED";
    case EnemyColor::GRAY: return "GRAY";
    case EnemyColor::PURPLE: return "PURPLE";
    case EnemyColor::WHITE: return "WHITE";
    default: return "UNKNOWN";
    }
}

// ============== Light Structure (for Traditional Backend) ==============

struct Light : public cv::RotatedRect {
    Light() = default;

    // Construct from contour
    explicit Light(const std::vector<cv::Point>& contour)
        : cv::RotatedRect(cv::minAreaRect(contour))
        , color(EnemyColor::WHITE) {
        if (contour.empty()) return;

        center = std::accumulate(
            contour.begin(), contour.end(), cv::Point2f(0, 0),
            [n = static_cast<float>(contour.size())](const cv::Point2f& a, const cv::Point& b) {
                return a + cv::Point2f(static_cast<float>(b.x) / n, static_cast<float>(b.y) / n);
            });

        cv::Point2f p[4];
        this->points(p);
        std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        top    = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width  = cv::norm(p[0] - p[1]);

        axis = top - bottom;
        if (cv::norm(axis) > 1e-6) {
            axis = axis / static_cast<float>(cv::norm(axis));
        }

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = static_cast<float>(tilt_angle / CV_PI * 180);
    }

    // Construct from top and bottom points
    explicit Light(const cv::Point2f& top_pt, const cv::Point2f& bottom_pt)
        : cv::RotatedRect()
        , color(EnemyColor::WHITE) {
        if (top_pt.y < bottom_pt.y) {
            top    = top_pt;
            bottom = bottom_pt;
        } else {
            top    = bottom_pt;
            bottom = top_pt;
        }
        center = (top + bottom) * 0.5f;

        length = cv::norm(top - bottom);
        if (length <= 1e-6) {
            length = 1e-6;
        }

        width = 1.0;

        axis = bottom - top;
        double an = cv::norm(axis);
        if (an > 1e-6)
            axis = axis / static_cast<float>(an);
        else
            axis = cv::Point2f(0.f, 1.f);

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = static_cast<float>(tilt_angle / CV_PI * 180.0);

        float angle_deg =
            static_cast<float>(std::atan2(bottom.y - top.y, bottom.x - top.x) * 180.0 / CV_PI);
        this->size  = cv::Size2f(static_cast<float>(width), static_cast<float>(length));
        this->angle = angle_deg;
    }

    EnemyColor color = EnemyColor::WHITE;
    cv::Point2f top{0, 0};
    cv::Point2f bottom{0, 0};
    cv::Point2f center{0, 0};
    cv::Point2f axis{0, 1};
    double length = 0;
    double width = 0;
    float tilt_angle = 0;
};

// ============== ArmorObject (Unified Detection Result) ==============

struct ArmorObject {
    cv::Rect2f rect;
    int cls    = -1;    // Number/class: 0=sentry, 1-5=numbers, 6=outpost, 7=base_small, 8=base_large
    int color  = -1;    // Color: 0=BLUE, 1=RED, 2=GRAY, 3=PURPLE
    float prob = 0.f;   // Confidence
    int area   = 0;
    cv::Point2f apex[4]{};  // 4 corners: top-left, top-right, bottom-right, bottom-left

    ArmorObject() = default;

    // Construct from two lights (for traditional backend)
    ArmorObject(const Light& left_light, const Light& right_light) {
        // Order: left-top, right-top, right-bottom, left-bottom
        apex[0] = left_light.top;
        apex[1] = right_light.top;
        apex[2] = right_light.bottom;
        apex[3] = left_light.bottom;

        // Calculate rect
        std::vector<cv::Point2f> pts = {apex[0], apex[1], apex[2], apex[3]};
        rect = cv::boundingRect(pts);

        // Calculate area
        area = static_cast<int>(cv::contourArea(pts));

        // Set color
        if (left_light.color == EnemyColor::BLUE || right_light.color == EnemyColor::BLUE) {
            color = 0;
        } else if (left_light.color == EnemyColor::RED || right_light.color == EnemyColor::RED) {
            color = 1;
        }
    }

    // Get center point
    cv::Point2f center() const {
        return (apex[0] + apex[1] + apex[2] + apex[3]) * 0.25f;
    }

    // Get armor type based on class
    ArmorType getArmorType() const {
        std::string str = armor_cls_to_string(cls);
        if (str == "1" || str == "base_large") {
            return ArmorType::LARGE;
        }
        return ArmorType::SMALL;
    }

    // Get 3D object points for PnP (in armor coordinate: X forward, Y left, Z up)
    std::vector<cv::Point3f> getObjectPoints() const {
        double w, h;
        if (getArmorType() == ArmorType::LARGE) {
            w = LARGE_ARMOR_WIDTH;
            h = LARGE_ARMOR_HEIGHT;
        } else {
            w = SMALL_ARMOR_WIDTH;
            h = SMALL_ARMOR_HEIGHT;
        }

        // Order: left-top, right-top, right-bottom, left-bottom
        return {
            cv::Point3f(0,  static_cast<float>(w/2),  static_cast<float>(h/2)),
            cv::Point3f(0, -static_cast<float>(w/2),  static_cast<float>(h/2)),
            cv::Point3f(0, -static_cast<float>(w/2), -static_cast<float>(h/2)),
            cv::Point3f(0,  static_cast<float>(w/2), -static_cast<float>(h/2))
        };
    }

    // Get image points (4 corners)
    std::vector<cv::Point2f> getImagePoints() const {
        return {apex[0], apex[1], apex[2], apex[3]};
    }
};

// ============== Additional Utility Functions ==============

inline bool isLargeArmor(const ArmorObject& armor) {
    return armor.getArmorType() == ArmorType::LARGE;
}

inline bool isLargeArmor(int cls) {
    auto str = armor_cls_to_string(cls);
    return (str == "1" || str == "base_large");
}

} // namespace rm_auto_aim
