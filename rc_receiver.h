#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

#include "esp_timer.h"
#include "driver/rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

#include "shared_types.h"
#include "platform_config.h"

namespace fc {
namespace rc {

// Interface chung
struct IRcReceiver {
    virtual bool begin() = 0;
    virtual bool read(fc::types::ControlInput& out) = 0;
    virtual bool hasFailsafe() const = 0;
    virtual ~IRcReceiver() = default;
};

/**
 * PwmReceiver (6 kênh) dùng RMT RX trên ESP32
 *
 *  - Pins (mặc định từ platform_config.h, truyền vào ctor):
 *      CH1 → 34 (Roll)
 *      CH2 → 35 (Pitch)
 *      CH3 → 32 (Throttle)
 *      CH4 → 33 (Yaw)
 *      CH5 → 25 (Arm)
 *      CH6 → 26 (Mode)
 *
 *  - Đặc tả:
 *      * Tần số frame ~50Hz
 *      * Xung cao ~1000..2000 µs
 *      * Idle (khoảng trống) giữa frame > 5000 µs (sync gap)
 *
 *  - Mapping:
 *      * CH1 → Roll   : 1000..2000 → [-1..+1]
 *      * CH2 → Pitch  : 1000..2000 → [-1..+1]
 *      * CH3 → Throt. : 1000..2000 → [ 0.. 1]
 *      * CH4 → Yaw    : 1000..2000 → [-1..+1]
 *      * CH5 → Arm    : >1500µs → true
 *      * CH6 → Mode   :
 *            <1300 → 0
 *            1300..1700 → 1
 *            >1700 → 2
 *
 *  - Failsafe:
 *      * Nếu mất xung >50ms trên các kênh 1..4 → _failsafe = true.
 *      * hasFailsafe() trả trạng thái này.
 */
class PwmReceiver : public IRcReceiver {
public:
    explicit PwmReceiver(const std::array<int,6>& pins);

    bool begin() override;
    bool read(fc::types::ControlInput& out) override;
    bool hasFailsafe() const override { return _failsafe; }

private:
    static constexpr uint32_t PULSE_MIN_US   = 1000;
    static constexpr uint32_t PULSE_MAX_US   = 2000;
    static constexpr uint32_t FAILSAFE_US    = 50000;  // 50 ms
    static constexpr uint32_t FRAME_GAP_US   = 5000;   // idle gap giữa frame
    static constexpr uint32_t GLITCH_US      = 50;     // deglitch 50us

    struct ChannelState {
        uint32_t lastPulseUs;   // micro giây đo được gần nhất
        uint32_t lastUpdateUs;  // esp_timer_get_time() khi nhận pulse
        bool     hasSignal;
    };

    std::array<int,6>         _pins;
    std::array<rmt_channel_t,6> _channels;
    std::array<ChannelState,6>  _ch;
    std::array<RingbufHandle_t,6> _ringbufs;

    bool _failsafe;

    static float mapPwmToNorm(int us, bool centerToZero);
    static uint8_t mapModeFromPwm(int us);

    void decodeChannel(int idx);
};

} // namespace rc
} // namespace fc
