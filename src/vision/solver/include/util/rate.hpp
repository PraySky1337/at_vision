#pragma once
#include <thread>
#include <chrono>
#include <stdexcept>

namespace util {

class Rate {
public:
    Rate() : initialized_(false) {}

    explicit Rate(double hz) {
        setFrequency(hz);
    }

    void tick() {
        if (!initialized_) {
            throw std::runtime_error("Rate not initialized. Call setFrequency() first.");
        }
        std::this_thread::sleep_until(next_time_);
        next_time_ += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period_);
    }

    void setFrequency(double hz) {
        period_    = std::chrono::duration<double>(1.0 / hz);
        next_time_ = std::chrono::steady_clock::now()
                   + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period_);
        initialized_ = true;
    }

private:
    std::chrono::duration<double> period_;
    std::chrono::steady_clock::time_point next_time_;
    bool initialized_;
};

} // namespace solver
