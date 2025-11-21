#pragma once
#include <opencv2/core.hpp>

namespace rm_auto_aim {

struct ArmorObject {
    cv::Rect2f rect;
    int cls    = -1;    // 数字/类别
    int color  = -1;    // 颜色
    float prob = 0.f;
    int area   = 0;
    cv::Point2f apex[4]{};
};

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

inline std::string armor_color_to_string(int color) {
    switch (color) {
    case 0: return "B"; // blue
    case 1: return "R"; // red
    case 2: return "G"; // gray
    case 3: return "P"; // purple
    default: return "UNKNOWN";
    }
}

inline bool isLargeArmor(const ArmorObject& armor) {
    auto str = armor_cls_to_string(armor.cls);
    if (str == "1" || str == "base_large")
        return true;
    else
        return false;
}


} // namespace rm_auto_aim