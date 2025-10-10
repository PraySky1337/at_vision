#pragma once
#include "packet.hpp"
#include <cstring>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <utility>

namespace rm_gimbal {
struct DeviceParser;
class Device {
public:
    explicit Device(DeviceParser& rhs);
    ~Device();
    bool open(uint16_t vid, uint16_t pid = 0) const;
    bool send_data(uint8_t* data, std::size_t size) const;
    void handle_events() const;

private:
    struct TransmitBuffer;
    class Impl;
    std::unique_ptr<Impl> impl_;
};

struct DeviceParser {
    using ParserFunc = std::function<void(const std::byte*, size_t)>;

    void parse(const std::byte* data, size_t size) const {
        /* ---------- 原有校验 ---------- */
        if (size < sizeof(HeaderFrame)) [[unlikely]] {
            RCLCPP_WARN(
                rclcpp::get_logger("gimbal_node"), "Frame size too small, expected %zu, got %zu",
                sizeof(HeaderFrame), size);
        }
        HeaderFrame header_frame;
        std::memcpy(&header_frame, data, sizeof(HeaderFrame));
        if (header_frame.sof != HeaderFrame::SoF()) {
            RCLCPP_WARN(
                rclcpp::get_logger("gimbal_node"), "Frame header invalid, expected %X, got %X",
                HeaderFrame::SoF(), header_frame.sof);
        }

        auto it = parser_table_.find(header_frame.id);
        if (it != parser_table_.end()) {
            it->second(data, size);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("gimbal_node"), "Unknown header %X", header_frame.sof);
            return;
        }
    }

    void register_parser(uint8_t id, ParserFunc func) { parser_table_[id] = std::move(func); }

private:
    std::unordered_map<uint8_t, ParserFunc> parser_table_;
};

} // namespace rm_gimbal
