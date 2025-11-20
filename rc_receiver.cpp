#include "rc_receiver.h"

namespace fc {
namespace rc {

PwmReceiver::PwmReceiver(const std::array<int,6>& pins)
    : _pins(pins)
    , _failsafe(true)
{
    // Gán kênh RMT: 6 kênh RX
    _channels = {
        RMT_CHANNEL_0,
        RMT_CHANNEL_1,
        RMT_CHANNEL_2,
        RMT_CHANNEL_3,
        RMT_CHANNEL_4,
        RMT_CHANNEL_5
    };

    for (int i = 0; i < 6; ++i) {
        _ch[i].lastPulseUs  = 1500;
        _ch[i].lastUpdateUs = static_cast<uint32_t>(esp_timer_get_time());
        _ch[i].hasSignal    = false;
        _ringbufs[i]        = nullptr;
    }
}

bool PwmReceiver::begin() {
    // Cấu hình 6 kênh RMT RX
    for (int i = 0; i < 6; ++i) {
        rmt_config_t cfg = {};
        cfg.channel = _channels[i];
        cfg.gpio_num = static_cast<gpio_num_t>(_pins[i]);
        cfg.clk_div = 80;                  // 1 tick = 1us (80MHz / 80)
        cfg.mem_block_num = 1;
        cfg.rmt_mode = RMT_MODE_RX;

        cfg.rx_config.filter_en = true;
        cfg.rx_config.filter_ticks_thresh = GLITCH_US; // ~50us deglitch
        cfg.rx_config.idle_threshold = FRAME_GAP_US;   // >5ms = frame gap

        if (rmt_config(&cfg) != ESP_OK) {
            return false;
        }
        if (rmt_driver_install(cfg.channel, 1000, 0) != ESP_OK) { // 1kB ringbuf
            return false;
        }
        if (rmt_get_ringbuf_handle(cfg.channel, &_ringbufs[i]) != ESP_OK) {
            return false;
        }
        if (_ringbufs[i] == nullptr) {
            return false;
        }
        if (rmt_rx_start(cfg.channel, true) != ESP_OK) {
            return false;
        }
    }

    _failsafe = true; // cho đến khi có xung thực
    return true;
}

// Map PWM [1000..2000]us -> [-1..+1] hoặc [0..1]
float PwmReceiver::mapPwmToNorm(int us, bool centerToZero) {
    if (us < static_cast<int>(PULSE_MIN_US)) us = PULSE_MIN_US;
    if (us > static_cast<int>(PULSE_MAX_US)) us = PULSE_MAX_US;

    if (centerToZero) {
        // 1000..2000 -> [-1..+1]
        float x = (static_cast<float>(us) - 1500.0f) / 500.0f;
        if (x < -1.0f) x = -1.0f;
        if (x >  1.0f) x =  1.0f;
        return x;
    } else {
        // 1000..2000 -> [0..1]
        float x = (static_cast<float>(us) - 1000.0f) / 1000.0f;
        if (x < 0.0f) x = 0.0f;
        if (x > 1.0f) x = 1.0f;
        return x;
    }
}

uint8_t PwmReceiver::mapModeFromPwm(int us) {
    if (us < 1300)  return 0;
    if (us <= 1700) return 1;
    return 2;
}

// Đọc non-blocking ringbuf cho 1 kênh, cập nhật lastPulseUs nếu có mẫu mới
void PwmReceiver::decodeChannel(int idx) {
    RingbufHandle_t rb = _ringbufs[idx];
    if (!rb) return;

    size_t length = 0;
    rmt_item32_t* items = (rmt_item32_t*) xRingbufferReceive(rb, &length, 0);
    if (!items || length == 0) {
        return;
    }

    int nItems = length / sizeof(rmt_item32_t);
    uint32_t now = static_cast<uint32_t>(esp_timer_get_time());

    // Mỗi frame servo ~1 high + 1 low (có thể nhiều item nếu noise).
    // Ta lấy item có level0==1 hoặc level1==1 đầu tiên đủ dài.
    for (int i = 0; i < nItems; ++i) {
        const rmt_item32_t& it = items[i];

        uint32_t pulse_us = 0;

        if (it.level0 == 1 && it.duration0 > 0) {
            pulse_us = it.duration0; // high trước
        } else if (it.level1 == 1 && it.duration1 > 0) {
            pulse_us = it.duration1; // high sau
        } else {
            continue;
        }

        // Bỏ qua các xung bất thường
        if (pulse_us < 500 || pulse_us > 2500) {
            continue;
        }

        _ch[idx].lastPulseUs  = pulse_us;
        _ch[idx].lastUpdateUs = now;
        _ch[idx].hasSignal    = true;
    }

    vRingbufferReturnItem(rb, (void*)items);
}

bool PwmReceiver::read(fc::types::ControlInput& out) {
    // Cập nhật measurement từ RMT cho tất cả kênh (non-blocking)
    for (int i = 0; i < 6; ++i) {
        decodeChannel(i);
    }

    uint32_t now = static_cast<uint32_t>(esp_timer_get_time());

    // Kiểm tra failsafe: nếu bất kỳ kênh 1..4 mất >50ms → failsafe
    bool fs = false;
    for (int i = 0; i < 4; ++i) { // CH1..CH4 là kênh quan trọng
        if (!_ch[i].hasSignal) {
            fs = true;
            break;
        }
        uint32_t dt = now - _ch[i].lastUpdateUs;
        if (dt > FAILSAFE_US) {
            fs = true;
            break;
        }
    }
    _failsafe = fs;

    // Điền output
    // Nếu failsafe → trả về giá trị an toàn
    if (_failsafe) {
        out.roll     = 0.0f;
        out.pitch    = 0.0f;
        out.yaw      = 0.0f;
        out.throttle = 0.0f;
        out.arm      = false;
        out.mode     = 0;
        out.stamp_us = now;
        return false;
    }

    int us_ch1 = static_cast<int>(_ch[0].lastPulseUs);
    int us_ch2 = static_cast<int>(_ch[1].lastPulseUs);
    int us_ch3 = static_cast<int>(_ch[2].lastPulseUs);
    int us_ch4 = static_cast<int>(_ch[3].lastPulseUs);
    int us_ch5 = static_cast<int>(_ch[4].lastPulseUs);
    int us_ch6 = static_cast<int>(_ch[5].lastPulseUs);

    out.roll     = mapPwmToNorm(us_ch1, true);
    out.pitch    = mapPwmToNorm(us_ch2, true);
    out.throttle = mapPwmToNorm(us_ch3, false);
    out.yaw      = mapPwmToNorm(us_ch4, true);

    out.arm      = (us_ch5 > 1500);
    out.mode     = mapModeFromPwm(us_ch6);

    out.stamp_us = now;
    return true;
}

} // namespace rc
} // namespace fc
