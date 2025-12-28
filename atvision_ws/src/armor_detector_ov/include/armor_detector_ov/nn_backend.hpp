// nn_backend.hpp
// Neural network detection backend with async pipeline
// Supports async queue-based processing with ROI optimization

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>

#include "armor_detector_ov/detector_backend.hpp"
#include "armor_detector_ov/ov_model_base.hpp"

namespace rm_auto_aim {

// Result returned by tryGetResult
struct DetectionResult {
    uint64_t frame_id = 0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::vector<ArmorObject> armors;
};

class NNBackend : public DetectorBackend {
public:
    struct Config {
        std::string model_name = "tup";
        std::string model_path;
        std::string device = "AUTO";
        std::string device_priorities = "GPU,CPU";
        bool enable_profiling = false;
        bool enable_multi_thread = false;

        // Pipeline config
        int queue_size = 4;       // Max pending frames
        int num_requests = 4;     // Number of inference requests
    };

    explicit NNBackend(const Config& config);
    ~NNBackend() override;

    // Initialize the backend
    bool init();

    // DetectorBackend interface
    std::vector<ArmorObject> detect(const cv::Mat& image,
                                    const cv::Rect& roi = {}) override;

    // Async queue interface
    uint64_t enqueue(const cv::Mat& image,
                     const cv::Rect& roi,
                     const rclcpp::Time& stamp) override;

    bool tryGetResult(uint64_t& frame_id,
                      rclcpp::Time& stamp,
                      std::vector<ArmorObject>& result) override;

    // Configuration
    void setDetectColor(int color) override { detect_color_ = color; }
    int getDetectColor() const override { return detect_color_; }

    // Status
    bool isInitialized() const override { return initialized_; }
    std::string getBackendName() const override { return "nn"; }

    // Capabilities
    bool supportsROI() const override { return true; }
    bool supportsAsync() const override { return true; }

    // Debug
    cv::Mat getDebugImage() const override { return debug_img_; }

    // Get execution devices
    std::vector<std::string> executionDevices() const;

private:
    // Job descriptor for input queue
    struct Job {
        uint64_t frame_id = 0;
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        cv::Mat image;           // Reference to original image
        cv::Rect roi;            // Requested ROI
        cv::Rect actual_roi;     // Actual ROI (clamped to image bounds)
    };

    // Slot for inference request pool
    struct Slot {
        ov::InferRequest request;
        std::unique_ptr<OVModelBase::PreprocContext> preproc_ctx;
        Job job;
        bool in_use = false;
    };

    // Worker threads
    void preprocessWorker();
    void postprocessWorker();

    // Pipeline stages
    void runPreprocess(Slot& slot);
    void runInference(Slot& slot);
    void runPostprocess(Slot& slot, std::vector<ArmorObject>& results);

    // Helper: compute actual ROI clamped to image bounds
    cv::Rect computeActualROI(const cv::Rect& roi, const cv::Size& image_size) const;

    // Helper: transform detection coordinates from ROI to full image
    void transformToFullImage(std::vector<ArmorObject>& armors,
                              const cv::Rect& actual_roi) const;

    // Inference completion callback
    void onInferComplete(size_t slot_idx, std::exception_ptr ex);

private:
    Config config_;
    int detect_color_ = 1;  // 0=BLUE, 1=RED
    bool initialized_ = false;

    // Model
    std::unique_ptr<OVModelBase> model_;

    // Slot pool
    std::vector<Slot> slots_;

    // Free slot indices
    std::mutex free_mtx_;
    std::condition_variable free_cv_;
    std::deque<size_t> free_slots_;

    // Input queue
    std::mutex input_mtx_;
    std::condition_variable input_cv_;
    std::deque<Job> input_queue_;
    bool input_closed_ = false;

    // Completed inference queue
    struct CompletedItem {
        size_t slot_idx;
        std::exception_ptr ex;
    };
    std::mutex done_mtx_;
    std::condition_variable done_cv_;
    std::deque<CompletedItem> done_queue_;
    bool done_closed_ = false;

    // Output result queue
    std::mutex output_mtx_;
    std::deque<DetectionResult> output_queue_;

    // Frame counter
    std::atomic<uint64_t> frame_counter_{0};

    // Worker threads
    std::thread preproc_thread_;
    std::thread postproc_thread_;
    std::atomic<bool> running_{false};

    // Debug image (for sync detect only)
    cv::Mat debug_img_;
};

}  // namespace rm_auto_aim
