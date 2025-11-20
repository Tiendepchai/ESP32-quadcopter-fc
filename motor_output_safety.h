
#pragma once

#include <cstdint>
#include <cstddef>

#include "shared_types.h"

namespace fc {
namespace safety {

class MotorOutputSafety {
public:
    MotorOutputSafety()
        : _minThrottleForSpin(0.05f)
        , _armed(false)
        , _softRampEnabled(false)
        , _rampThrottle(0.0f)
        , _rampRatePerSec(1.0f)
    {}

    void setArmed(bool armed) {
        _armed = armed;
        if (!armed) {
            _rampThrottle = 0.0f;
        }
    }

    void setMinThrottleForSpin(float t) {
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        _minThrottleForSpin = t;
    }

    void enableSoftRamp(bool en) {
        _softRampEnabled = en;
        _rampThrottle = en ? 0.0f : 1.0f;
    }

    void setSoftRampRate(float ratePerSec) {
        if (ratePerSec < 0.0f) ratePerSec = 0.0f;
        _rampRatePerSec = ratePerSec;
    }

    void applySafety(const fc::types::ControlInput& rc,
                     fc::types::MotorCommand& motors)
    {
        if (!_armed || !rc.arm) {
            motors.m1 = motors.m2 = motors.m3 = motors.m4 = 0.0f;
            _rampThrottle = 0.0f;
            return;
        }

        if (rc.throttle < _minThrottleForSpin) {
            motors.m1 = motors.m2 = motors.m3 = motors.m4 = 0.0f;
        }

        motors.m1 = clamp01(motors.m1);
        motors.m2 = clamp01(motors.m2);
        motors.m3 = clamp01(motors.m3);
        motors.m4 = clamp01(motors.m4);
    }

    void applySoftRamp(const fc::types::ControlInput& rc,
                       fc::types::MotorCommand& motors,
                       float dt_s)
    {
        if (!_softRampEnabled) return;
        if (!_armed || !rc.arm) {
            _rampThrottle = 0.0f;
            return;
        }
        if (dt_s <= 0.0f) return;

        _rampThrottle += _rampRatePerSec * dt_s;
        if (_rampThrottle > 1.0f) _rampThrottle = 1.0f;
        if (rc.throttle < _minThrottleForSpin) _rampThrottle = 0.0f;

        float k = clamp01(_rampThrottle);
        motors.m1 = clamp01(motors.m1 * k);
        motors.m2 = clamp01(motors.m2 * k);
        motors.m3 = clamp01(motors.m3 * k);
        motors.m4 = clamp01(motors.m4 * k);
    }

private:
    static float clamp01(float x) {
        if (x < 0.0f) return 0.0f;
        if (x > 1.0f) return 1.0f;
        return x;
    }

    float _minThrottleForSpin;
    bool  _armed;

    bool  _softRampEnabled;
    float _rampThrottle;
    float _rampRatePerSec;
};

} // namespace safety
} // namespace fc
