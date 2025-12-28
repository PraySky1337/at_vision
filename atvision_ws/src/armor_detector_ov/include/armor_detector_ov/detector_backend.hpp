// detector_backend.hpp
// Abstract detector backend interface
// Supports both synchronous (Traditional) and asynchronous queue-based (NN) detection

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>

#include "armor_detector_ov/armor_types.hpp"

namespace rm_auto_aim {

class DetectorBackend {
public:
    virtual ~DetectorBackend() = default;

    // ============ Synchronous Interface (TraditionalBackend) ============

    // Synchronous detection - returns immediately with results
    // roi parameter is ignored by backends that don't support ROI
    virtual std::vector<ArmorObject> detect(const cv::Mat& image,
                                            const cv::Rect& roi = {}) = 0;

    // ============ Asynchronous Queue Interface (NNBackend) ============

    // Enqueue an image for async processing
    // Returns frame_id for later result matching
    // Default implementation returns 0 (async not supported)
    virtual uint64_t enqueue(const cv::Mat& image,
                             const cv::Rect& roi,
                             const rclcpp::Time& stamp) {
        (void)image;
        (void)roi;
        (void)stamp;
        return 0;
    }

    // Try to get a completed result (non-blocking)
    // Returns true if result available, false otherwise
    // frame_id and stamp identify which frame the result belongs to
    virtual bool tryGetResult(uint64_t& frame_id,
                              rclcpp::Time& stamp,
                              std::vector<ArmorObject>& result) {
        (void)frame_id;
        (void)stamp;
        (void)result;
        return false;
    }

    // ============ Capability Queries ============

    // Whether this backend supports ROI-based detection
    virtual bool supportsROI() const { return false; }

    // Whether this backend supports async queue interface
    virtual bool supportsAsync() const { return false; }

    // ============ Configuration ============

    // Set target detection color (0=BLUE, 1=RED)
    virtual void setDetectColor(int color) = 0;
    virtual int getDetectColor() const = 0;

    // ============ Status ============

    virtual bool isInitialized() const = 0;
    virtual std::string getBackendName() const = 0;

    // ============ Debug ============

    virtual cv::Mat getDebugImage() const { return {}; }
    virtual cv::Mat getBinaryImage() const { return {}; }
};

}  // namespace rm_auto_aim
