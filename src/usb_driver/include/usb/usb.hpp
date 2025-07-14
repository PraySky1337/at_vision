#pragma once
#include "packet.hpp"
#include <cstring>
#include <functional>
#include <logger/logger.hpp>
#include <memory>

namespace usb_driver {
struct DeviceParser;
class Device {
public:
    Device(DeviceParser& rhs);
    ~Device();
    bool open(uint16_t vid, uint16_t pid = 0);
    void send_data(uint8_t* data, size_t size);
    void handle_events();
    void stop_handling_events();

private:
    struct TransmitBuffer;
    class Impl;
    std::unique_ptr<Impl> impl_;
    std::unique_ptr<TransmitBuffer> transmit_buffer_;
};

struct DeviceParser {
    using ParserFunc = std::function<void(const std::byte*, size_t)>;

    void parse(const std::byte* data, size_t size) {
        /* ---------- 原有校验 ---------- */
        if (size < sizeof(HeaderFrame)) [[unlikely]] {
            ATLOG_WARN("Frame size too small, expected {:X}, got {:X}", sizeof(HeaderFrame), size);
        }
        HeaderFrame header_frame;
        std::memcpy(&header_frame, data, sizeof(HeaderFrame));
        if (header_frame.sof != HeaderFrame::SoF()) {
            ATLOG_WARN("Frame header invalid, expected {:X}, got {:X}", HeaderFrame::SoF(), size);
        }

        /* ---------- 查表分发 ---------- */
        auto it = parser_table_.find(header_frame.id);
        if (it != parser_table_.end()) {
            it->second(data, size);
        } else {
            ATLOG_WARN("Unknown header {:X}", header_frame.sof);
            return;
        }

        /* ---------- FPS 统计 ---------- */
        static std::size_t cnt = 0;
        static auto t0         = std::chrono::steady_clock::now();
        constexpr std::chrono::seconds window{5};

        ++cnt;
        auto now = std::chrono::steady_clock::now();
        if (now - t0 >= window) {
            double fps =
                static_cast<double>(cnt)
                / std::chrono::duration_cast<std::chrono::duration<double>>(now - t0).count();
            ATLOG_INFO("Parser FPS = {:.1f}", fps);
            cnt = 0;
            t0  = now;
        }
    }

    void register_parser(uint8_t id, ParserFunc func) { parser_table_[id] = func; }

private:
    std::unordered_map<uint8_t, ParserFunc> parser_table_;
};

} // namespace usb_driver
