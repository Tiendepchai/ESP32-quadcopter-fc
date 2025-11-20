
#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>

#include "platform_config.h"
#include "shared_types.h"

namespace fc {
namespace est {

struct IAttitudeEstimator {
    virtual void reset(float roll_deg,float pitch_deg,float yaw_deg) = 0;
    virtual void update(const fc::types::ImuSample& s,float dt_s) = 0;
    virtual fc::types::FusedAttitude get() const = 0;
    virtual ~IAttitudeEstimator() = default;
};

class ComplementaryFilter : public IAttitudeEstimator {
public:
    ComplementaryFilter()
        : _roll_deg(0.0f), _pitch_deg(0.0f), _yaw_deg(0.0f)
        , _alpha(0.0f)
        , _tau_s(0.5f)
        , _dt_s(fc::cfg::LOOP_DT_S)
        , _stamp_us(0)
        , _initialized(false)
        , _roll_acc_lpf_deg(0.0f)
        , _pitch_acc_lpf_deg(0.0f)
        , _acc_lpf_alpha(0.8f)
    {
        updateAlpha();
    }

    void reset(float roll_deg,float pitch_deg,float yaw_deg) override {
        _roll_deg  = roll_deg;
        _pitch_deg = pitch_deg;
        _yaw_deg   = yaw_deg;
        _roll_acc_lpf_deg  = roll_deg;
        _pitch_acc_lpf_deg = pitch_deg;
        _initialized = true;
    }

    void setTimeConstant(float tau_s) {
        if (tau_s <= 0.0f) tau_s = 0.01f;
        _tau_s = tau_s;
        updateAlpha();
    }

    void setDt(float dt_s) {
        if (dt_s <= 0.0f) dt_s = fc::cfg::LOOP_DT_S;
        _dt_s = dt_s;
        updateAlpha();
    }

    void setAccelLpfAlpha(float a) {
        if (a < 0.0f) a = 0.0f;
        if (a > 1.0f) a = 1.0f;
        _acc_lpf_alpha = a;
    }

    void update(const fc::types::ImuSample& s,float dt_s) override {
        if (dt_s <= 0.0f) dt_s = _dt_s;

        constexpr float RAD2DEG = 57.29577951308232f;

        const float ax = s.ax;
        const float ay = s.ay;
        const float az = s.az;

        const float roll_acc_rad  = std::atan2(ay, az);
        const float pitch_acc_rad = std::atan2(-ax, std::sqrt(ay * ay + az * az));

        float roll_acc_deg  = roll_acc_rad * RAD2DEG;
        float pitch_acc_deg = pitch_acc_rad * RAD2DEG;

        if (!_initialized) {
            _roll_deg          = roll_acc_deg;
            _pitch_deg         = pitch_acc_deg;
            _yaw_deg           = 0.0f;
            _roll_acc_lpf_deg  = roll_acc_deg;
            _pitch_acc_lpf_deg = pitch_acc_deg;
            _initialized       = true;
        } else {
            _roll_acc_lpf_deg  = _acc_lpf_alpha * _roll_acc_lpf_deg
                               + (1.0f - _acc_lpf_alpha) * roll_acc_deg;
            _pitch_acc_lpf_deg = _acc_lpf_alpha * _pitch_acc_lpf_deg
                               + (1.0f - _acc_lpf_alpha) * pitch_acc_deg;
        }

        const float gx = s.gx;
        const float gy = s.gy;
        const float gz = s.gz;

        _roll_deg  += gx * dt_s;
        _pitch_deg += gy * dt_s;
        _yaw_deg   += gz * dt_s;

        _roll_deg  = _alpha * _roll_deg  + (1.0f - _alpha) * _roll_acc_lpf_deg;
        _pitch_deg = _alpha * _pitch_deg + (1.0f - _alpha) * _pitch_acc_lpf_deg;

        _stamp_us = s.stamp_us;
    }

    fc::types::FusedAttitude get() const override {
        fc::types::FusedAttitude att;
        att.roll_deg  = _roll_deg;
        att.pitch_deg = _pitch_deg;
        att.yaw_deg   = _yaw_deg;
        att.stamp_us  = _stamp_us;
        return att;
    }

private:
    void updateAlpha() {
        const float denom = _tau_s + _dt_s;
        if (denom <= 0.0f) _alpha = 0.0f;
        else _alpha = _tau_s / denom;
    }

    float _roll_deg;
    float _pitch_deg;
    float _yaw_deg;

    float _alpha;
    float _tau_s;
    float _dt_s;

    uint32_t _stamp_us;
    bool _initialized;

    float _roll_acc_lpf_deg;
    float _pitch_acc_lpf_deg;
    float _acc_lpf_alpha;
};

} // namespace est
} // namespace fc
