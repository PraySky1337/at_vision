// nn_backend.cpp
// Neural network detection backend with async pipeline

#include "armor_detector_ov/nn_backend.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>

namespace rm_auto_aim {

NNBackend::NNBackend(const Config& config)
    : config_(config) {}

NNBackend::~NNBackend() {
    // Signal threads to stop
    running_.store(false, std::memory_order_release);

    // Close input queue and wake up preproc worker
    {
        std::lock_guard<std::mutex> lk(input_mtx_);
        input_closed_ = true;
    }
    input_cv_.notify_all();
    free_cv_.notify_all();

    // Join preproc thread
    if (preproc_thread_.joinable()) {
        preproc_thread_.join();
    }

    // Wait for any in-flight inference to complete
    for (auto& slot : slots_) {
        if (slot.in_use) {
            try {
                slot.request.wait();
            } catch (...) {}
        }
    }

    // Close done queue and wake up postproc worker
    {
        std::lock_guard<std::mutex> lk(done_mtx_);
        done_closed_ = true;
    }
    done_cv_.notify_all();

    // Join postproc thread
    if (postproc_thread_.joinable()) {
        postproc_thread_.join();
    }
}

bool NNBackend::init() {
    // Create model from registry
    model_ = OVModelManager::instance().create(config_.model_name);
    if (!model_) {
        std::cerr << "[NNBackend] Failed to create model: " << config_.model_name << std::endl;
        return false;
    }

    // Load model
    if (!model_->load(config_.model_path, config_.device,
                      config_.enable_profiling, config_.enable_multi_thread,
                      config_.device_priorities)) {
        std::cerr << "[NNBackend] Failed to load model: " << config_.model_path << std::endl;
        model_.reset();
        return false;
    }

    // Create slot pool
    const int num_requests = std::max(1, config_.num_requests);
    slots_.reserve(num_requests);
    for (int i = 0; i < num_requests; ++i) {
        Slot slot;
        slot.request = model_->createInferRequest();

        // Set up async callback
        const size_t slot_idx = static_cast<size_t>(i);
        slot.request.set_callback(
            [this, slot_idx](std::exception_ptr ex) {
                this->onInferComplete(slot_idx, ex);
            });

        slots_.push_back(std::move(slot));
        free_slots_.push_back(slot_idx);
    }

    // Start worker threads
    running_.store(true, std::memory_order_release);
    preproc_thread_ = std::thread(&NNBackend::preprocessWorker, this);
    postproc_thread_ = std::thread(&NNBackend::postprocessWorker, this);

    initialized_ = true;
    return true;
}

std::vector<ArmorObject> NNBackend::detect(const cv::Mat& image, const cv::Rect& roi) {
    // Synchronous detection - direct inference without pipeline
    if (!initialized_ || !model_) {
        return {};
    }

    // Compute actual ROI
    cv::Rect actual_roi = computeActualROI(roi, image.size());

    // Get ROI image (view, no copy)
    cv::Mat roi_img = image(actual_roi);

    // Direct inference using model's sync API
    std::vector<ArmorObject> results;
    model_->infer(roi_img, results);

    // Transform coordinates back to full image
    transformToFullImage(results, actual_roi);

    // Store debug image
    debug_img_ = image.clone();

    return results;
}

uint64_t NNBackend::enqueue(const cv::Mat& image,
                            const cv::Rect& roi,
                            const rclcpp::Time& stamp) {
    if (!initialized_ || !running_.load(std::memory_order_acquire)) {
        return 0;
    }

    uint64_t frame_id = frame_counter_.fetch_add(1, std::memory_order_relaxed) + 1;

    Job job;
    job.frame_id = frame_id;
    job.stamp = stamp;
    job.image = image;  // Shallow copy - caller must ensure image lifetime
    job.roi = roi;
    job.actual_roi = computeActualROI(roi, image.size());

    {
        std::lock_guard<std::mutex> lk(input_mtx_);
        if (input_closed_) {
            return 0;
        }

        // Drop oldest if queue is full (keep latency bounded)
        if (static_cast<int>(input_queue_.size()) >= config_.queue_size) {
            input_queue_.pop_front();
        }

        input_queue_.push_back(std::move(job));
    }
    input_cv_.notify_one();

    return frame_id;
}

bool NNBackend::tryGetResult(uint64_t& frame_id,
                             rclcpp::Time& stamp,
                             std::vector<ArmorObject>& result) {
    std::lock_guard<std::mutex> lk(output_mtx_);
    if (output_queue_.empty()) {
        return false;
    }

    DetectionResult det = std::move(output_queue_.front());
    output_queue_.pop_front();

    frame_id = det.frame_id;
    stamp = det.stamp;
    result = std::move(det.armors);
    return true;
}

std::vector<std::string> NNBackend::executionDevices() const {
    if (!model_) {
        return {};
    }
    return model_->executionDevices();
}

void NNBackend::preprocessWorker() {
    while (running_.load(std::memory_order_acquire)) {
        // Wait for a job
        Job job;
        {
            std::unique_lock<std::mutex> lk(input_mtx_);
            input_cv_.wait(lk, [this]() {
                return input_closed_ || !input_queue_.empty();
            });

            if (input_queue_.empty()) {
                if (input_closed_) break;
                continue;
            }

            job = std::move(input_queue_.front());
            input_queue_.pop_front();
        }

        // Acquire a free slot
        size_t slot_idx = 0;
        {
            std::unique_lock<std::mutex> lk(free_mtx_);
            free_cv_.wait(lk, [this]() {
                return !running_.load(std::memory_order_acquire) || !free_slots_.empty();
            });

            if (free_slots_.empty()) {
                break;
            }

            slot_idx = free_slots_.front();
            free_slots_.pop_front();
        }

        auto& slot = slots_[slot_idx];
        slot.in_use = true;
        slot.job = std::move(job);
        slot.preproc_ctx.reset();

        // Run preprocessing
        runPreprocess(slot);

        if (!slot.preproc_ctx) {
            // Preprocessing failed, release slot and push empty result
            slot.in_use = false;
            {
                std::lock_guard<std::mutex> lk(free_mtx_);
                free_slots_.push_back(slot_idx);
            }
            free_cv_.notify_one();

            DetectionResult result;
            result.frame_id = slot.job.frame_id;
            result.stamp = slot.job.stamp;
            {
                std::lock_guard<std::mutex> lk(output_mtx_);
                output_queue_.push_back(std::move(result));
            }
            continue;
        }

        // Start async inference
        runInference(slot);
    }
}

void NNBackend::onInferComplete(size_t slot_idx, std::exception_ptr ex) {
    if (!running_.load(std::memory_order_relaxed)) {
        return;
    }

    CompletedItem item;
    item.slot_idx = slot_idx;
    item.ex = std::move(ex);

    {
        std::lock_guard<std::mutex> lk(done_mtx_);
        if (done_closed_) {
            return;
        }
        done_queue_.push_back(std::move(item));
    }
    done_cv_.notify_one();
}

void NNBackend::postprocessWorker() {
    while (running_.load(std::memory_order_acquire)) {
        // Wait for completed inference
        CompletedItem item;
        {
            std::unique_lock<std::mutex> lk(done_mtx_);
            done_cv_.wait(lk, [this]() {
                return done_closed_ || !done_queue_.empty();
            });

            if (done_queue_.empty()) {
                if (done_closed_) break;
                continue;
            }

            item = std::move(done_queue_.front());
            done_queue_.pop_front();
        }

        auto& slot = slots_[item.slot_idx];

        // Run postprocessing
        std::vector<ArmorObject> results;
        if (!item.ex && slot.preproc_ctx) {
            runPostprocess(slot, results);
        }

        // Build result
        DetectionResult det_result;
        det_result.frame_id = slot.job.frame_id;
        det_result.stamp = slot.job.stamp;
        det_result.armors = std::move(results);

        // Release slot
        slot.in_use = false;
        slot.preproc_ctx.reset();
        slot.job = Job{};

        {
            std::lock_guard<std::mutex> lk(free_mtx_);
            free_slots_.push_back(item.slot_idx);
        }
        free_cv_.notify_one();

        // Push result to output queue
        {
            std::lock_guard<std::mutex> lk(output_mtx_);
            output_queue_.push_back(std::move(det_result));
        }
    }
}

void NNBackend::runPreprocess(Slot& slot) {
    if (slot.job.image.empty()) {
        return;
    }

    // Get ROI image (view, no copy)
    cv::Mat roi_img = slot.job.image(slot.job.actual_roi);

    // Call model's preprocess
    slot.preproc_ctx = model_->preprocess(roi_img, slot.request);
}

void NNBackend::runInference(Slot& slot) {
    try {
        slot.request.start_async();
    } catch (const std::exception& e) {
        std::cerr << "[NNBackend] Inference start failed: " << e.what() << std::endl;
        onInferComplete(static_cast<size_t>(&slot - slots_.data()),
                        std::make_exception_ptr(e));
    }
}

void NNBackend::runPostprocess(Slot& slot, std::vector<ArmorObject>& results) {
    model_->postprocess(*slot.preproc_ctx, slot.request, results);

    // Transform detection coordinates from ROI to full image
    transformToFullImage(results, slot.job.actual_roi);
}

cv::Rect NNBackend::computeActualROI(const cv::Rect& roi,
                                     const cv::Size& image_size) const {
    if (roi.width <= 0 || roi.height <= 0) {
        return cv::Rect(0, 0, image_size.width, image_size.height);
    }

    // Clamp ROI to image bounds
    cv::Rect clamped = roi & cv::Rect(0, 0, image_size.width, image_size.height);

    if (clamped.width <= 0 || clamped.height <= 0) {
        return cv::Rect(0, 0, image_size.width, image_size.height);
    }

    return clamped;
}

void NNBackend::transformToFullImage(std::vector<ArmorObject>& armors,
                                     const cv::Rect& actual_roi) const {
    if (actual_roi.x == 0 && actual_roi.y == 0) {
        return;  // No offset needed
    }

    const float offset_x = static_cast<float>(actual_roi.x);
    const float offset_y = static_cast<float>(actual_roi.y);

    for (auto& armor : armors) {
        // Transform bounding rect
        armor.rect.x += offset_x;
        armor.rect.y += offset_y;

        // Transform apex points
        for (auto& pt : armor.apex) {
            pt.x += offset_x;
            pt.y += offset_y;
        }
    }
}

}  // namespace rm_auto_aim
