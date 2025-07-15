#pragma once
#include <cmath>
#include <limits>
#include <optional>

namespace core::util {
inline std::optional<float> safe_double_to_float(double x) noexcept {
    // NaN 和 ±Inf 直接按 IEEE 转
    if (std::isnan(x) || std::isinf(x))
        return static_cast<float>(x);

    constexpr double f_max = std::numeric_limits<float>::max();
    constexpr double f_min = -f_max;

    if (x > f_max || x < f_min)
        return std::nullopt;      // 溢出，放弃转换

    return static_cast<float>(x); // 范围 OK
};
} // namespace core::util