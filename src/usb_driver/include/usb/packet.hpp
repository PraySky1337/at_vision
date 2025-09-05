#pragma once
#include <cstdint>
#include <iomanip>
#include <iostream>

namespace usb_driver {
#pragma pack(1)
struct HeaderFrame {
    static constexpr uint8_t SoF() { return 0x5A; }

    static constexpr uint8_t EoF() { return 0xA5; }

    uint8_t sof; // 0x5A
    uint8_t len;
    uint8_t id;  // 0x01
};

struct ReceiveImuData {
    HeaderFrame header;
    uint32_t time_stamp;
    struct {
        uint8_t self_color;
        float yaw;
        float pitch;
        float roll;
        float yaw_vel;
        float pitch_vel;
        float roll_vel;
    } data;
    uint8_t eof; // 0xA5
};

struct SendVisionData {
    HeaderFrame header;
    struct {
        uint8_t fire  : 1;
        uint8_t reserved : 7;
        float target_yaw;
        float target_pitch;
    } data;
    uint8_t eof;
};

#pragma pack()

inline constexpr const char* colorName(uint8_t c) {
    return c == 0 ? "RED" : c == 1 ? "BLUE" : "UNKNOWN";
}
inline const char* color_name(uint8_t c) { return c == 0 ? "RED" : c == 1 ? "BLUE" : "UNKNOWN"; }

inline void print_imu(const ReceiveImuData& imu) {
    constexpr double RAD2DEG = 180.0 / 3.1415926535;

    std::cout << "\n---------------- IMU PACKET ----------------\n";
    std::cout << "stamp        : " << imu.time_stamp << " ms\n";
    std::cout << "self_color   : " << int(imu.data.self_color) << " ("
              << color_name(imu.data.self_color) << ")\n";
    //   std::cout << "vision_open  : " << (imu.data.vision_open ? "YES" : "NO")
    // << '\n';

    std::cout << std::fixed << std::setprecision(3);

    std::cout << "\nEuler (rad)  : "
              << "yaw " << std::setw(8) << imu.data.yaw << ' ' << "pitch " << std::setw(8)
              << imu.data.pitch << ' ' << "roll " << std::setw(8) << imu.data.roll << '\n';

    std::cout << "Euler (deg)  : "
              << "yaw " << std::setw(8) << imu.data.yaw * RAD2DEG << ' ' << "pitch " << std::setw(8)
              << imu.data.pitch * RAD2DEG << ' ' << "roll " << std::setw(8)
              << imu.data.roll * RAD2DEG << '\n';

    std::cout << "\nAngular vel  : "
              << "yaw " << std::setw(8) << imu.data.yaw_vel << " rad/s  "
              << "pitch " << std::setw(8) << imu.data.pitch_vel << " rad/s  "
              << "roll " << std::setw(8) << imu.data.roll_vel << " rad/s\n";

    std::cout << "--------------------------------------------\n";
}

} // namespace usb_driver