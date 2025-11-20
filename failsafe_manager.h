
#pragma once

#include <cstdint>
#include <cstddef>

#include "esp_timer.h"
#include "shared_types.h"
#include "rc_receiver.h"

namespace fc {
namespace safety {

class FailsafeManager {
public:
    explicit FailsafeManager(fc::rc::IRcReceiver& rx)
        : _rx(rx)
        , _lastOkUs(static_cast<uint32_t>(esp_timer_get_time()))
        , _active(false)
    {}

    void update() {
        uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
        if (!_rx.hasFailsafe()) {
            _lastOkUs = now;
            if (_active) _active = false;
        } else {
            uint32_t dt = now - _lastOkUs;
            if (dt > FAILSAFE_THRESHOLD_US) {
                _active = true;
            }
        }
    }

    bool isActive() const { return _active; }

    void applyFailsafe(fc::types::ControlInput& rc,
                       fc::types::MotorCommand& motors)
    {
        if (!_active) return;
        rc.arm      = false;
        rc.throttle = 0.0f;
        motors.m1 = motors.m2 = motors.m3 = motors.m4 = 0.0f;
    }

private:
    static constexpr uint32_t FAILSAFE_THRESHOLD_US = 300000; // 300 ms

    fc::rc::IRcReceiver& _rx;
    uint32_t _lastOkUs;
    bool     _active;
};

} // namespace safety
} // namespace fc
