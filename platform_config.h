
#pragma once

#include <cstdint>
#include "driver/uart.h"

// ==== Feature flags ====
// 0 = tắt Web PID tuner, 1 = bật
#ifndef PID_tune_websocket
#define PID_tune_websocket 0
#endif

// Alias chuẩn hóa để dùng trong code
#define PID_TUNE_WEBSOCKET PID_tune_websocket

// WiFi cho PID tuner (dùng khi PID_TUNE_WEBSOCKET = 1)
#define PID_TUNE_WIFI_SSID "ESP32_PID_TUNER"
#define PID_TUNE_WIFI_PASS "12345678"

namespace fc {
namespace cfg {

// Loop frequency & timing
constexpr int   LOOP_HZ   = 500;                 // vòng lặp chính 500 Hz
constexpr float LOOP_DT_S = 1.0f / LOOP_HZ;      // 0.002s

// ESC PWM frequency
constexpr int PWM_ESC_FREQ_HZ_DEFAULT  = 400;    // Hz
constexpr int PWM_ESC_FREQ_HZ_FALLBACK = 50;     // Hz (servo-style)

// Motor pins (Quad X, Betaflight order)
//  M1 = Front-Right (CW)
//  M2 = Rear-Right  (CCW)
//  M3 = Rear-Left   (CW)
//  M4 = Front-Left  (CCW)
constexpr int MOT1_PIN = 13;   // Front-Right (CW)
constexpr int MOT2_PIN = 12;   // Rear-Right (CCW)  // CẢNH BÁO: GPIO12 là strap MTDI
constexpr int MOT3_PIN = 14;   // Rear-Left (CW)
constexpr int MOT4_PIN = 27;   // Front-Left (CCW)

// PWM range for ESC (microseconds)
constexpr int PWM_MIN_US = 1000;
constexpr int PWM_MAX_US = 2000;

// CẢNH BÁO:
//  - GPIO12 (MTDI) là pin strap, phải giữ mức an toàn (tương ứng duty MIN) lúc boot.
//  - Nếu gặp lỗi boot do ESC kéo cao, nên dời M2 sang GPIO4 hoặc GPIO5.

// RC backend selection
enum RcBackend {
    RC_BACKEND_PWM  = 1,
    RC_BACKEND_SBUS = 2
};

// Chọn backend ở đây (mặc định PWM 6 kênh)
constexpr RcBackend RC_BACKEND = RC_BACKEND_PWM;

// RC PWM 6 kênh (nếu RC_BACKEND == RC_BACKEND_PWM)
constexpr int RC_CH1_PIN = 34; // Roll (input-only)
constexpr int RC_CH2_PIN = 35; // Pitch (input-only)
constexpr int RC_CH3_PIN = 32; // Throttle
constexpr int RC_CH4_PIN = 33; // Yaw
constexpr int RC_CH5_PIN = 25; // Arm/AUX1
constexpr int RC_CH6_PIN = 26; // Mode/AUX2;

// SBUS (nếu RC_BACKEND == RC_BACKEND_SBUS)
constexpr uart_port_t SBUS_UART  = UART_NUM_1;
constexpr int         SBUS_RX_PIN = 34;
constexpr int         SBUS_BAUD   = 100000;
constexpr bool        SBUS_INVERT = true;
// Dùng cấu hình 8E2 cho UART (8 bit, Even parity, 2 stop bit).

// IMU & GPS & phụ trợ
constexpr int      I2C_SDA_PIN  = 21;
constexpr int      I2C_SCL_PIN  = 22;
constexpr uint32_t I2C_FREQ_HZ  = 400000; // 400kHz

constexpr uart_port_t GPS_UART  = UART_NUM_2;
constexpr int         GPS_RX_PIN = 16;
constexpr int         GPS_TX_PIN = 17;
constexpr int         GPS_BAUD   = 115200;

constexpr int VBAT_ADC_PIN = 36;   // ADC1_CH0
// Ghi chú chia áp: 100k (trên) : 22k (dưới)
//  Vbat = Vadc * (Rtop + Rbottom) / Rbottom  ≈ Vadc * 5.545

constexpr int LED_PIN    = 15;
constexpr int BUZZER_PIN = 2;

// Static assert / warning about GPIO12
static_assert(MOT2_PIN == 12,
              "Warning: MOT2_PIN is GPIO12 (MTDI strap). "
              "Ensure ESC does not pull this pin high at boot, "
              "or move MOT2 to GPIO4/GPIO5.");

} // namespace cfg
} // namespace fc
