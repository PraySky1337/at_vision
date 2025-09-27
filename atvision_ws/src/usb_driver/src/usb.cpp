#include "usb/usb.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <libusb-1.0/libusb.h>
#include "usb/logger.hpp"
#include <stdexcept>
#include <sys/types.h>
#include <thread>

/* ------------------- error ------------------- */
namespace usb_driver {
template <class F>
class FinalAction {
public:
    explicit FinalAction(F f)
        : f_(f)
        , enabled_(true) {}
    ~FinalAction() {
        if (enabled_)
            f_();
    }
    void disable() { enabled_ = false; }

    FinalAction(const FinalAction&)            = delete;
    FinalAction& operator=(const FinalAction&) = delete;
    FinalAction(FinalAction&&)                 = delete;
    FinalAction& operator=(FinalAction&&)      = delete;

private:
    F f_;
    bool enabled_;
};

class Device::Impl {
public:
    static constexpr uint16_t VID          = 0x0483; // STM32
    static constexpr uint8_t EP_OUT        = 0x01;
    static constexpr uint8_t EP_IN         = 0x81;
    static constexpr uint8_t INTERFACE_NUM = 1;

    // ---------- 自定义异常 ----------
    struct error : std::runtime_error {
        explicit error(int code)
            : std::runtime_error(libusb_error_name(code))
            , code(code) {}
        int code;
    };

    explicit Impl(DeviceParser& parser)
        : device_parser_(parser) {}
    ~Impl() {
        handling_events_ = false;
        cleanup();
        if (ctx_)
            libusb_exit(ctx_);
    }

    // ==========================================================
    // 公开接口
    // ==========================================================
    bool open(uint16_t vid, uint16_t pid = 0);
    void process_once(); // handle_events() 调用的封装
    bool sync_send(uint8_t* data, std::size_t size, unsigned tout_ms = 500);

    // ==========================================================
    // Hot‑plug 回调
    // ==========================================================
    void on_hotplug(libusb_hotplug_event ev);

    // ==========================================================
    // 内部工具
    // ==========================================================
    uint16_t find_device(uint16_t vid);
    void alloc_transfer();
    void submit_transfer();
    void cleanup(); // 关闭/释放资源
    bool try_reopen();

    // ==========================================================
    // 成员变量
    // ==========================================================
    DeviceParser& device_parser_;

    libusb_context* ctx_          = nullptr;
    libusb_device_handle* handle_ = nullptr;
    libusb_transfer* rx_transfer_ = nullptr;
    libusb_hotplug_callback_handle hp_handle_{};

    std::byte rx_buf_[64]{};

    std::atomic_bool handling_events_{false};
    std::atomic_bool disconnected_{false};
    bool first_rx_{true};
};

// ============================================================================
// Impl 实现
// ============================================================================
bool Device::Impl::open(uint16_t vid, uint16_t pid) {
    if (libusb_init(&ctx_) != 0)
        throw error(LIBUSB_ERROR_OTHER);

    if (pid == 0)
        pid = find_device(vid);

    // 打开设备
    handle_ = libusb_open_device_with_vid_pid(ctx_, vid, pid);
    if (!handle_)
        throw error(LIBUSB_ERROR_NO_DEVICE);

    if (libusb_kernel_driver_active(handle_, INTERFACE_NUM))
        libusb_detach_kernel_driver(handle_, INTERFACE_NUM);

    if (libusb_claim_interface(handle_, INTERFACE_NUM))
        throw error(LIBUSB_ERROR_BUSY);

    // 分配 bulk IN transfer
    alloc_transfer();

    // Hot‑plug 回调
    if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        int rc = libusb_hotplug_register_callback(
            ctx_,
            static_cast<libusb_hotplug_event>(
                LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT | LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED),
            LIBUSB_HOTPLUG_NO_FLAGS, vid, pid, LIBUSB_HOTPLUG_MATCH_ANY,
            [](libusb_context* /*ctx*/, libusb_device* /*dev*/, libusb_hotplug_event event,
               void* user) -> int {
                static_cast<Impl*>(user)->on_hotplug(event);
                return 0;
            },
            this, &hp_handle_);
        if (rc != LIBUSB_SUCCESS)
            ATLOG_WARN("Hot‑plug callback reg failed: {}", libusb_error_name(rc));
    }

    submit_transfer(); // 启动异步接收
    handling_events_ = true;
    return true;
}

// 查找 PID (若仅给 VID)
uint16_t Device::Impl::find_device(uint16_t vid) {
    libusb_device** list;
    ssize_t cnt = libusb_get_device_list(ctx_, &list);
    if (cnt < 0)
        throw error(static_cast<int>(cnt));
    FinalAction free{[&]() { libusb_free_device_list(list, 1); }};

    std::vector<uint16_t> pids;
    for (ssize_t i = 0; i < cnt; ++i) {
        libusb_device_descriptor d{};
        if (libusb_get_device_descriptor(list[i], &d) == 0 && d.idVendor == vid)
            pids.push_back(d.idProduct);
    }
    if (pids.empty())
        throw error(LIBUSB_ERROR_NOT_FOUND);
    if (pids.size() > 1)
        throw error(LIBUSB_ERROR_OVERFLOW);
    return pids[0];
}

// 分配 bulk transfer
void Device::Impl::alloc_transfer() {
    rx_transfer_ = libusb_alloc_transfer(0);
    if (!rx_transfer_)
        throw error(LIBUSB_ERROR_NO_MEM);

    libusb_fill_bulk_transfer(
        rx_transfer_, handle_, EP_IN, reinterpret_cast<uint8_t*>(rx_buf_), sizeof(rx_buf_),
        [](libusb_transfer* tr) {
            auto self = static_cast<Impl*>(tr->user_data);
            if (!self->handling_events_)
                return;

            // 设备中途中断
            if (tr->status != LIBUSB_TRANSFER_COMPLETED) {
                self->disconnected_ = true;
                return;
            }
            int len = tr->actual_length;
            if (len > 0) {
                if (self->first_rx_) {
                    self->first_rx_ = false;
                } else {
                    self->device_parser_.parse(
                        reinterpret_cast<std::byte*>(tr->buffer), static_cast<size_t>(len));
                }
            }

            // 继续提交
            int rc = libusb_submit_transfer(tr);
            if (rc != 0)
                self->disconnected_ = true;
        },
        this, 0);
}

// 提交 transfer
void Device::Impl::submit_transfer() {
    if (libusb_submit_transfer(rx_transfer_) != 0)
        throw error(LIBUSB_ERROR_IO);
}

// 清理所有资源
void Device::Impl::cleanup() {
    if (rx_transfer_) {
        libusb_cancel_transfer(rx_transfer_);
        libusb_free_transfer(rx_transfer_);
        rx_transfer_ = nullptr;
    }
    if (handle_) {
        libusb_release_interface(handle_, INTERFACE_NUM);
        libusb_close(handle_);
        handle_ = nullptr;
    }
}

// 事件循环里调用 — 单线程
void Device::Impl::process_once() {
    timeval tv{0, 0};
    libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);

    if (disconnected_) {
        cleanup();
        try_reopen();
    }
}

// 热插拔回调
void Device::Impl::on_hotplug(libusb_hotplug_event ev) {
    if (ev == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        disconnected_ = true;
    } else if (ev == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        if (!handle_) {
            try_reopen();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

// 重连
bool Device::Impl::try_reopen() {
    while (handling_events_) {
        try {
            if (open(VID)) {
                disconnected_ = false;
                ATLOG_INFO("USB re‑connected");
                return true;
            }
        } catch (...) {}
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
}

// 同步发送
bool Device::Impl::sync_send(uint8_t* data, std::size_t size, unsigned tout_ms) {
    if (!handle_)
        return false;
    int actual = 0;
    int rc = libusb_bulk_transfer(handle_, EP_OUT, data, static_cast<int>(size), &actual, tout_ms);
    return rc == 0 && actual == static_cast<int>(size);
}

// ============================================================
// Device 封装
// ============================================================
Device::Device(DeviceParser& p)
    : impl_(std::make_unique<Impl>(p)) {}
Device::~Device() = default;

bool Device::open(uint16_t vid, uint16_t pid) { return impl_->open(vid, pid); }
bool Device::send_data(uint8_t* d, std::size_t s) { return impl_->sync_send(d, s); }
void Device::handle_events() { impl_->process_once(); }

} // namespace usb_driver
