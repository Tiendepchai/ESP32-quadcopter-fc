
#pragma once

#include <cstdint>
#include <cstddef>

#include <Arduino.h>
#include "driver/ledc.h"

#include "platform_config.h"
#include "shared_types.h"

namespace fc {
namespace esc {

struct IEscDriver {
    virtual bool begin() = 0;
    virtual void write(const fc::types::MotorCommand& cmd) = 0; // input [0..1]
    virtual void arm() = 0;
    virtual void disarm() = 0;
    virtual ~IEscDriver() = default;
};

class EscPwm : public IEscDriver {
public:
    EscPwm()
        : _timer(LEDC_TIMER_0)
        , _speedMode(LEDC_HIGH_SPEED_MODE)
        , _freqHz(fc::cfg::PWM_ESC_FREQ_HZ_DEFAULT)
        , _dutyResolution(13)
        , _dutyMax((1u << 13) - 1u)
        , _armed(false)
    {
        _pins[0] = fc::cfg::MOT1_PIN;
        _pins[1] = fc::cfg::MOT2_PIN;
        _pins[2] = fc::cfg::MOT3_PIN;
        _pins[3] = fc::cfg::MOT4_PIN;

        _channels[0] = LEDC_CHANNEL_0;
        _channels[1] = LEDC_CHANNEL_1;
        _channels[2] = LEDC_CHANNEL_2;
        _channels[3] = LEDC_CHANNEL_3;
    }

    bool begin() override {
        ledc_timer_config_t tconf = {};
        tconf.speed_mode      = _speedMode;
        tconf.duty_resolution = static_cast<ledc_timer_bit_t>(_dutyResolution);
        tconf.timer_num       = _timer;
        tconf.freq_hz         = _freqHz;
        tconf.clk_cfg         = LEDC_AUTO_CLK;

        esp_err_t err = ledc_timer_config(&tconf);
        if (err != ESP_OK) {
            _freqHz     = fc::cfg::PWM_ESC_FREQ_HZ_FALLBACK;
            tconf.freq_hz = _freqHz;
            if (ledc_timer_config(&tconf) != ESP_OK) {
                return false;
            }
        }
        _dutyMax = (1u << _dutyResolution) - 1u;

        for (int i = 0; i < 4; ++i) {
            ledc_channel_config_t cconf = {};
            cconf.speed_mode = _speedMode;
            cconf.channel    = _channels[i];
            cconf.timer_sel  = _timer;
            cconf.intr_type  = LEDC_INTR_DISABLE;
            cconf.gpio_num   = static_cast<gpio_num_t>(_pins[i]);
            cconf.duty       = 0;
            cconf.hpoint     = 0;
            if (ledc_channel_config(&cconf) != ESP_OK) {
                return false;
            }
        }

        // CẢNH BÁO: GPIO12 (MOT2_PIN) là strap MTDI – cần giữ duty MIN khi boot.
        uint32_t dutyMin = usToDuty(fc::cfg::PWM_MIN_US);
        for (int i = 0; i < 4; ++i) {
            ledc_set_duty(_speedMode,_channels[i],dutyMin);
            ledc_update_duty(_speedMode,_channels[i]);
        }
        _armed = false;
        return true;
    }

    void write(const fc::types::MotorCommand& cmd) override {
        auto clamp01 = [](float v){
            if (v < 0.0f) return 0.0f;
            if (v > 1.0f) return 1.0f;
            return v;
        };
        uint32_t dutyMin = usToDuty(fc::cfg::PWM_MIN_US);

        if (!_armed) {
            for (int i = 0; i < 4; ++i) {
                ledc_set_duty(_speedMode,_channels[i],dutyMin);
                ledc_update_duty(_speedMode,_channels[i]);
            }
            return;
        }

        float m1 = clamp01(cmd.m1);
        float m2 = clamp01(cmd.m2);
        float m3 = clamp01(cmd.m3);
        float m4 = clamp01(cmd.m4);

        int span_us = fc::cfg::PWM_MAX_US - fc::cfg::PWM_MIN_US;
        int us1 = fc::cfg::PWM_MIN_US + static_cast<int>(m1 * span_us + 0.5f);
        int us2 = fc::cfg::PWM_MIN_US + static_cast<int>(m2 * span_us + 0.5f);
        int us3 = fc::cfg::PWM_MIN_US + static_cast<int>(m3 * span_us + 0.5f);
        int us4 = fc::cfg::PWM_MIN_US + static_cast<int>(m4 * span_us + 0.5f);

        uint32_t d1 = usToDuty(us1);
        uint32_t d2 = usToDuty(us2);
        uint32_t d3 = usToDuty(us3);
        uint32_t d4 = usToDuty(us4);

        ledc_set_duty(_speedMode,_channels[0],d1); ledc_update_duty(_speedMode,_channels[0]);
        ledc_set_duty(_speedMode,_channels[1],d2); ledc_update_duty(_speedMode,_channels[1]);
        ledc_set_duty(_speedMode,_channels[2],d3); ledc_update_duty(_speedMode,_channels[2]);
        ledc_set_duty(_speedMode,_channels[3],d4); ledc_update_duty(_speedMode,_channels[3]);
    }

    void arm() override {
        uint32_t dutyMin = usToDuty(fc::cfg::PWM_MIN_US);
        for (int i = 0; i < 4; ++i) {
            ledc_set_duty(_speedMode,_channels[i],dutyMin);
            ledc_update_duty(_speedMode,_channels[i]);
        }
        _armed = true;
    }

    void disarm() override {
        uint32_t dutyMin = usToDuty(fc::cfg::PWM_MIN_US);
        for (int i = 0; i < 4; ++i) {
            ledc_set_duty(_speedMode,_channels[i],dutyMin);
            ledc_update_duty(_speedMode,_channels[i]);
        }
        _armed = false;
    }

private:
    uint32_t usToDuty(int us) const {
        if (us < fc::cfg::PWM_MIN_US) us = fc::cfg::PWM_MIN_US;
        if (us > fc::cfg::PWM_MAX_US) us = fc::cfg::PWM_MAX_US;
        int span_us = fc::cfg::PWM_MAX_US - fc::cfg::PWM_MIN_US;
        float norm = static_cast<float>(us - fc::cfg::PWM_MIN_US) / static_cast<float>(span_us);
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        uint32_t duty = static_cast<uint32_t>(norm * static_cast<float>(_dutyMax) + 0.5f);
        if (duty > _dutyMax) duty = _dutyMax;
        return duty;
    }

    ledc_timer_t _timer;
    ledc_mode_t  _speedMode;
    uint32_t     _freqHz;
    uint8_t      _dutyResolution;
    uint32_t     _dutyMax;
    bool         _armed;

    int            _pins[4];
    ledc_channel_t _channels[4];
};

// =================== ESC CALIBRATOR (OPTIONAL) ===================
//
// Quy trình kinh điển (nhưng còn tùy loại ESC):
//  1) Gửi xung MAX (2ms) một lúc → ESC beep báo nhận max throttle.
//  2) Gửi xung MIN (1ms) → ESC beep báo nhận min throttle.
//  3) Lưu lại dải 1–2ms.
//
// Lưu ý:
//  - THÁO CÁNH QUẠT trước khi chạy.
//  - Không chạm tay gần motor/ESC trong khi calibrate.
//  - Có thể tất cả ESC sẽ beep cùng lúc (vì dùng chung tín hiệu).
//
class EscCalibrator {
public:
    // maxMs / minMs: thời gian giữ max/min (ms)
    void runCalibration(IEscDriver& esc,
                        uint32_t maxMs = 3000,
                        uint32_t minMs = 3000)
    {
        Serial.println(F("=== ESC CALIBRATION START ==="));
        Serial.println(F(">>> THAO TOAN BO CANH QUAT NGAY LAP TUC <<<"));
        Serial.println(F("Quy trinh: MAX ~3s -> MIN ~3s"));
        Serial.println(F("DANG GUI LENH..."));

        // Đảm bảo ESC đang ở trạng thái an toàn
        esc.disarm();
        delay(500);

        // ARM ESC, nhưng ta sẽ tự gửi lệnh max/min
        esc.arm();
        delay(500);

        fc::types::MotorCommand cmd{};

        // 1) MAX throttle (1.0) cho 4 motor
        cmd.m1 = cmd.m2 = cmd.m3 = cmd.m4 = 1.0f;
        esc.write(cmd);
        Serial.println(F("[ESC CAL] MAX throttle..."));
        delay(maxMs);

        // 2) MIN throttle (0.0) - tương ứng PWM_MIN_US
        cmd.m1 = cmd.m2 = cmd.m3 = cmd.m4 = 0.0f;
        esc.write(cmd);
        Serial.println(F("[ESC CAL] MIN throttle..."));
        delay(minMs);

        // Đưa ESC về trạng thái disarm + MIN
        esc.disarm();
        Serial.println(F("=== ESC CALIBRATION DONE ==="));
        Serial.println(F("Ngat pin, doi 5s, sau do bat lai de bay binh thuong."));
    }
};

} // namespace esc
} // namespace fc
