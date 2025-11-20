
#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>

#include "shared_types.h"
#include "rc_receiver.h"

namespace fc {
namespace rc {

class RcInput {
public:
    explicit RcInput(IRcReceiver& rx)
        : _rx(rx)
        , _deadband(0.03f)
        , _expoRP(0.3f)
        , _expoYaw(0.2f)
        , _slewLimitPerSec(0.0f)
        , _hasPrev(false)
        , _prevStampUs(0)
    {
        _prev.roll = _prev.pitch = _prev.yaw = _prev.throttle = 0.0f;
        _prev.arm = false;
        _prev.mode = 0;
        _prev.stamp_us = 0;
    }

    void setDeadband(float db) {
        if (db < 0.0f) db = 0.0f;
        if (db > 0.5f) db = 0.5f;
        _deadband = db;
    }

    void setExpo(float expoRP,float expoYaw) {
        _expoRP  = clamp01(expoRP);
        _expoYaw = clamp01(expoYaw);
    }

    void setSlewLimit(float slewratePerSec) {
        if (slewratePerSec < 0.0f) slewratePerSec = 0.0f;
        _slewLimitPerSec = slewratePerSec;
    }

    bool getLatest(fc::types::ControlInput& out) {
        fc::types::ControlInput tmp{};
        bool ok = _rx.read(tmp);

        if (!ok || _rx.hasFailsafe()) {
            out.roll = out.pitch = out.yaw = 0.0f;
            out.throttle = 0.0f;
            out.arm = false;
            out.mode = 0;
            out.stamp_us = tmp.stamp_us;
            _hasPrev = false;
            return false;
        }

        float roll  = tmp.roll;
        float pitch = tmp.pitch;
        float yaw   = tmp.yaw;
        float thr   = tmp.throttle;

        applyDeadband(roll);
        applyDeadband(pitch);
        applyDeadband(yaw);

        roll  = applyExpo(roll,  _expoRP);
        pitch = applyExpo(pitch, _expoRP);
        yaw   = applyExpo(yaw,   _expoYaw);

        uint32_t now_us = tmp.stamp_us;
        if (!_hasPrev) {
            _prev = tmp;
            _prev.roll  = roll;
            _prev.pitch = pitch;
            _prev.yaw   = yaw;
            _prevStampUs = now_us;
            _hasPrev = true;
        } else {
            float dt_s = 0.0f;
            if (now_us > _prevStampUs) {
                dt_s = (now_us - _prevStampUs) * 1e-6f;
            }
            if (_slewLimitPerSec > 0.0f && dt_s > 0.0f) {
                float maxDelta = _slewLimitPerSec * dt_s;
                roll  = applySlewLimit(_prev.roll,  roll,  maxDelta);
                pitch = applySlewLimit(_prev.pitch, pitch, maxDelta);
                yaw   = applySlewLimit(_prev.yaw,   yaw,   maxDelta);
            }
            _prev.roll  = roll;
            _prev.pitch = pitch;
            _prev.yaw   = yaw;
            _prev.throttle = thr;
            _prev.arm   = tmp.arm;
            _prev.mode  = tmp.mode;
            _prev.stamp_us = now_us;
            _prevStampUs   = now_us;
        }

        out.roll     = roll;
        out.pitch    = pitch;
        out.yaw      = yaw;
        out.throttle = thr;
        out.arm      = tmp.arm;
        out.mode     = tmp.mode;
        out.stamp_us = now_us;
        return true;
    }

private:
    IRcReceiver& _rx;
    float _deadband;
    float _expoRP;
    float _expoYaw;
    float _slewLimitPerSec;
    fc::types::ControlInput _prev;
    bool _hasPrev;
    uint32_t _prevStampUs;

    static float clamp01(float x) {
        if (x < 0.0f) return 0.0f;
        if (x > 1.0f) return 1.0f;
        return x;
    }
    static float clampN1P1(float x) {
        if (x < -1.0f) return -1.0f;
        if (x >  1.0f) return  1.0f;
        return x;
    }

    void applyDeadband(float& x) const {
        float db = _deadband;
        float ax = std::fabs(x);
        if (ax < db) {
            x = 0.0f;
        } else {
            float sign = (x > 0.0f) ? 1.0f : -1.0f;
            x = (ax - db) / (1.0f - db) * sign;
        }
    }

    float applyExpo(float x,float expo) const {
        float x3 = x * x * x;
        return clampN1P1(x3 * expo + x * (1.0f - expo));
    }

    static float applySlewLimit(float prev,float target,float maxDelta) {
        float delta = target - prev;
        if (delta > maxDelta) delta = maxDelta;
        else if (delta < -maxDelta) delta = -maxDelta;
        return clampN1P1(prev + delta);
    }
};

} // namespace rc
} // namespace fc
