#include "usb/usb.hpp"

#include <atomic>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <libusb-1.0/libusb.h>
#include <logger/logger.hpp>
#include <sys/types.h>

/* ------------------- error ------------------- */
namespace usb_driver {
class Device::Impl {
public:
    struct error : std::runtime_error {
        explicit error(int code)
            : std::runtime_error(libusb_error_name(code))
            , code(code) {}
        int code;
    };

    Impl(DeviceParser& rhs)
        : device_parser_(rhs) {}
    ~Impl() {
        stop_handling_events();
        libusb_cancel_transfer(receive_transfer_);        // (2)
        libusb_free_transfer(receive_transfer_);          // (3)
        libusb_release_interface(handle_, INTERFACE_NUM); // (4)
        libusb_close(handle_);                            // (5)
        libusb_exit(libusb_context_);                     // (6)
    }

    void handle_events() {
        // Synchronously send a connect packet to clear upward buffer, reset alarm and
        // configuration
        const unsigned char connect[] = {0x81, 0x00};
        int actual_length;
        if (libusb_bulk_transfer(
                handle_, EP_OUT, const_cast<unsigned char*>(connect), sizeof(connect),
                &actual_length, 500)
            != 0) {
            ATLOG_ERROR("Failed to send connect packet.");
            throw error(LIBUSB_ERROR_IO);
        }

        // Asynchronous reception
        if (libusb_submit_transfer(receive_transfer_) != 0) {
            ATLOG_ERROR("Failed to submit receive transfer.");
            throw error(LIBUSB_ERROR_IO);
        }
        handling_events_.store(true, std::memory_order::relaxed);
        receive_transfer_busy_ = true;
        handle_events_thread_  = std::thread([this]() {
            while (receive_transfer_busy_) {
                libusb_handle_events(libusb_context_);
            }
        });
    }

    void stop_handling_events() {
        handling_events_ = false;
        if (handle_events_thread_.joinable()) {
            handle_events_thread_.join();
        }
    }

    uint16_t find_device(int vid) {
        libusb_device** list;
        ssize_t count = libusb_get_device_list(libusb_context_, &list);
        if (count < 0) {
            throw error(static_cast<int>(count));
        }
        FinalAction free_list{[&list]() { libusb_free_device_list(list, 1); }};
        std::vector<uint16_t> product_ids;

        for (ssize_t i = 0; i < count; ++i) {
            libusb_device* dev = list[i];
            libusb_device_descriptor desc;
            if (libusb_get_device_descriptor(dev, &desc) != 0) {
                continue;
            }
            if (desc.idVendor == vid) {
                product_ids.push_back(desc.idProduct);
            }
        }
        if (product_ids.size() == 0) {
            ATLOG_ERROR("No devices found with VID={:#06x}.", vid);
            throw error(LIBUSB_ERROR_NOT_FOUND);
        } else if (product_ids.size() == 1) {
            return product_ids[0]; // Return the single product ID found
        } else if (product_ids.size() > 1) {
            ATLOG_ERROR("Multiple devices found with VID={:#06x}. Please specify PID.", vid);
            for (size_t i = 0; i < product_ids.size(); ++i) {
                ATLOG_ERROR("Device {}: PID={:#06x}", i + 1, product_ids[i]);
            }
            throw error(LIBUSB_ERROR_NOT_FOUND);
        }
        return 0;                  // Should not reach here, but return 0 if no device found
    }

    bool open(uint16_t vid, uint16_t pid = 0) {
        if (int code = libusb_init(&libusb_context_)) {
            ATLOG_ERROR("Failed to initialize libusb: {}", libusb_error_name(code));
            throw error(code);
        }
        if (pid == 0) {
            pid = find_device(vid);
        }
        FinalAction exit_libusb{[this]() {
            if (libusb_context_) {
                libusb_exit(libusb_context_);
                libusb_context_ = nullptr;
            }
        }};
        handle_ = libusb_open_device_with_vid_pid(libusb_context_, vid, pid);
        if (!handle_) {
            throw error(LIBUSB_ERROR_NO_DEVICE);
        }
        FinalAction close_device{[this]() {
            if (handle_) {
                libusb_close(handle_);
                handle_ = nullptr;
            }
        }};
        if (libusb_kernel_driver_active(handle_, INTERFACE_NUM)) {
            libusb_detach_kernel_driver(handle_, INTERFACE_NUM);
        }
        if (libusb_claim_interface(handle_, INTERFACE_NUM)) {
            throw error(LIBUSB_ERROR_BUSY);
        }
        FinalAction release_interface{[this]() {
            if (handle_) {
                libusb_release_interface(handle_, INTERFACE_NUM);
            }
        }};
        // ring_buffer_.emplace_back();
        receive_transfer_ = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(
            receive_transfer_, handle_, EP_IN, reinterpret_cast<u_char*>(receive_buffer_),
            sizeof(receive_buffer_),
            [](libusb_transfer* transfer) {
                static_cast<Impl*>(transfer->user_data)->receive_callback(transfer);
            },
            this, 0);
        if (!receive_transfer_) {
            throw error(LIBUSB_ERROR_NO_MEM);
        }
        close_device.disable();
        release_interface.disable();
        exit_libusb.disable();
        return true;
    }

    void LIBUSB_CALL receive_callback(libusb_transfer* transfer) {
        if (!handling_events_.load(std::memory_order::relaxed)) [[unlikely]] {
            receive_transfer_busy_ = false;
            return;
        }
        FinalAction resubmit_transfer{[transfer]() {
            int ret = libusb_submit_transfer(transfer);
            if (ret != 0) {
                if (ret == LIBUSB_ERROR_NO_DEVICE) [[unlikely]] {
                    ATLOG_ERROR("Failed to re-submit receive transfer: Device disconnected. ");
                } else {
                    ATLOG_ERROR("Failed to re-submit receive transfer: {}", ret);
                }
                std::terminate();
            }
        }};
        if (first_reception) {
            first_reception = false;
            return;
        }

        if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
            ATLOG_ERROR("USB receive error: Transfer not completed!");
            return;
        }

        int got = transfer->actual_length;
        if (got <= 0) {
            ATLOG_ERROR("USB receive error: No data received!");
            return;
        }
        device_parser_.parse(
            reinterpret_cast<const std::byte*>(transfer->buffer), static_cast<size_t>(got));
    }

    template <typename Functor>
    struct FinalAction {
        constexpr explicit FinalAction(Functor clean)
            : clean_{clean}
            , enabled_(true) {}

        constexpr FinalAction(const FinalAction&)            = delete;
        constexpr FinalAction& operator=(const FinalAction&) = delete;
        constexpr FinalAction(FinalAction&&)                 = delete;
        constexpr FinalAction& operator=(FinalAction&&)      = delete;

        ~FinalAction() {
            if (enabled_)
                clean_();
        }

        void disable() { enabled_ = false; };

    private:
        Functor clean_;
        bool enabled_;
    };

    constexpr static const uint16_t VID          = 0x0483; // Vendor ID for STM32
    constexpr static const char EP_OUT           = 0x01;
    constexpr static const char EP_IN            = 0x81;
    constexpr static const uint8_t INTERFACE_NUM = 1;
    DeviceParser& device_parser_;
    libusb_transfer* receive_transfer_{nullptr};
    libusb_device_handle* handle_{nullptr};
    libusb_context* libusb_context_{nullptr};
    std::byte receive_buffer_[64]{};
    std::thread handle_events_thread_;
    std::atomic_bool receive_transfer_busy_{false};
    std::atomic_bool handling_events_{false};
    bool first_reception{true};
};

struct Device::TransmitBuffer {
    static constexpr std::size_t BUF_SZ = 64;

    bool add_transmission(const uint8_t* data, std::size_t size) {
        assert(size < BUF_SZ);
        if (size > BUF_SZ)
            return false;

        libusb_transfer* tr = nullptr;
        // TODO

        std::memcpy(tr->buffer, data, size);
        tr->length = static_cast<int>(size);

        int ret = libusb_submit_transfer(tr);
        if (ret != 0) {
            ATLOG_ERROR("submit_transfer: {}", libusb_error_name(ret));
            return false;
        }
        return true;
    }

    /* ---------------- 回调：放回空闲池 ---------------- */
    void LIBUSB_CALL usb_transmit_complete_callback(libusb_transfer* tr) {
        if (tr->status != LIBUSB_TRANSFER_COMPLETED) [[unlikely]] {
            ATLOG_WARN("Usb transfer not completed");
        }
    }

private:
};

Device::Device(DeviceParser& device_parser)
    : impl_(std::make_unique<Impl>(device_parser)) {}
Device::~Device() { this->stop_handling_events(); }

bool Device::open(uint16_t vid, uint16_t pid) { return impl_->open(vid, pid); }

void Device::handle_events() { impl_->handle_events(); }

void Device::stop_handling_events() { impl_->stop_handling_events(); }

void Device::send_data(uint8_t* data, size_t size) {
    transmit_buffer_->add_transmission(data, size);
}

} // namespace usb_driver