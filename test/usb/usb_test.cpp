#include <libusb-1.0/libusb.h>
#include <iostream>
#include <iomanip>
#include <vector>

int main() {
    libusb_context* ctx = nullptr;
    libusb_device_handle* h = nullptr;
    const uint16_t VID = 0x0483, PID = 0x5740;
    const uint8_t  EP_IN = 0x81;   // 端点号：IN 方向一般为 0x81
    const uint8_t  EP_OUT = 0x01;  // 若需写数据则用它

    libusb_init(&ctx);
    // libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, 3);

    h = libusb_open_device_with_vid_pid(ctx, VID, PID);
    if (!h) {
        std::cerr << "open fail\n";
        return 1;
    }

    if (libusb_kernel_driver_active(h, 0) == 1)
        libusb_detach_kernel_driver(h, 0);

    int r = libusb_claim_interface(h, 0);
    if (r < 0) {
        std::cerr << "claim fail: " << libusb_error_name(r) << "\n";
        return 1;
    }
    std::cout << "claim ok, start reading...\n";

    // 循环读取裸报文
    std::vector<unsigned char> buf(256);
    while (true) {
        int actual = 0;
        int ret = libusb_bulk_transfer(h, EP_IN, buf.data(), buf.size(), &actual, 1000);
        if (ret == LIBUSB_SUCCESS && actual > 0) {
            std::cout << "recv (" << actual << "): ";
            for (int i = 0; i < actual; ++i)
                std::cout << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<int>(buf[i]) << " ";
            std::cout << std::dec << std::endl;
        } else if (ret == LIBUSB_ERROR_TIMEOUT) {
            continue; // 没数据正常超时
        } else {
            std::cerr << "read error: " << libusb_error_name(ret) << std::endl;
            break;
        }
    }

    libusb_release_interface(h, 0);
    libusb_close(h);
    libusb_exit(ctx);
    return 0;
}
