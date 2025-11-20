
#pragma once

#include <cstdint>
#include <cstddef>

namespace fc {
namespace types {

struct FusedAttitude {
    // Attitude hợp nhất:
    // roll_deg, pitch_deg, yaw_deg: độ, quy ước chuẩn
    // (right-hand, nose right / nose up / yaw CCW nhìn từ trên).
    float roll_deg;   // [deg]
    float pitch_deg;  // [deg]
    float yaw_deg;    // [deg]
    // Dấu thời gian micro giây từ esp_timer_get_time().
    uint32_t stamp_us; // [us]
};

struct ImuSample {
    // Gia tốc: m/s^2 trong hệ tọa độ body.
    float ax;  // [m/s^2]
    float ay;  // [m/s^2]
    float az;  // [m/s^2]
    // Gyro: deg/s trong hệ body.
    float gx;  // [deg/s]
    float gy;  // [deg/s]
    float gz;  // [deg/s]
    uint32_t stamp_us; // [us]
};

struct ControlInput {
    // Lệnh từ RC sau chuẩn hóa:
    // roll, pitch, yaw: [-1..+1]
    // throttle: [0..1]
    float roll;      // [-1..+1]
    float pitch;     // [-1..+1]
    float yaw;       // [-1..+1]
    float throttle;  // [0..1]
    // Cờ arm/disarm từ kênh AUX.
    bool arm;
    // Mode: 0/1/2... (ví dụ: 0=ANGLE, 1=ACRO, 2=ALTHOLD...)
    uint8_t mode;
    uint32_t stamp_us; // [us]
};

struct MotorCommand {
    // Lệnh động cơ chuẩn hóa [0..1], sau đó sẽ map sang micro giây.
    float m1; // Front-Right (CW)
    float m2; // Rear-Right (CCW)
    float m3; // Rear-Left (CW)
    float m4; // Front-Left (CCW)
};

struct BatteryState {
    float vbat;    // [V]
    float current; // [A]
    uint32_t stamp_us; // [us]
};

struct GpsData {
    double   lat;                // [deg]
    double   lon;                // [deg]
    float    alt_m;              // [m]
    float    ground_speed_mps;   // [m/s]
    float    hdop;               // HDOP (dimensionless)
    bool     fix;                // true nếu fix 3D/2D hợp lệ
    uint8_t  sats;               // số vệ tinh
    uint32_t stamp_us;           // [us]
};

} // namespace types
} // namespace fc
