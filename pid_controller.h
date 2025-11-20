
#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>

#include "filter_dsp.h"

namespace fc {
namespace ctl {

class PidController {
public:
    struct Result {
        float out;
        float p;
        float i;
        float d;
    };

    PidController()
        : _kp(0.0f), _ki(0.0f), _kd(0.0f)
        , _dt(0.002f)
        , _iTerm(0.0f)
        , _prevMeas(0.0f)
        , _outMin(-1.0f), _outMax(1.0f)
        , _iMin(-1.0f), _iMax(1.0f)
    {
        _dLpf.reset(0.0f);
        float fs = 1.0f / _dt;
        _dLpf.configure(200.0f, fs);
    }

    void setGains(float kp,float ki,float kd) {
        _kp = kp; _ki = ki; _kd = kd;
    }

    void setDt(float dt) {
        if (dt <= 0.0f) dt = 0.001f;
        _dt = dt;
    }

    void setOutputLimits(float outMin,float outMax) {
        _outMin = outMin; _outMax = outMax;
        if (_outMin > _outMax) {
            float t = _outMin; _outMin = _outMax; _outMax = t;
        }
    }

    void setIntegralLimits(float iMin,float iMax) {
        _iMin = iMin; _iMax = iMax;
        if (_iMin > _iMax) {
            float t = _iMin; _iMin = _iMax; _iMax = t;
        }
    }

    void setDtermLpf(float fc_hz) {
        float fs = (_dt > 0.0f) ? (1.0f / _dt) : 1000.0f;
        if (fc_hz <= 0.0f) {
            fc_hz = fs * 0.49f;
        }
        _dLpf.configure(fc_hz, fs);
    }

    Result compute(float meas, float setpoint) {
        Result res{0.0f,0.0f,0.0f,0.0f};

        const float error = setpoint - meas;

        float pTerm = _kp * error;

        _iTerm += _ki * error * _dt;
        _iTerm  = clamp(_iTerm,_iMin,_iMax);

        float dTerm = 0.0f;
        if (_dt > 0.0f && _kd != 0.0f) {
            float dMeas = (meas - _prevMeas) / _dt;
            float dRaw  = -_kd * dMeas;
            dTerm = _dLpf.process(dRaw);
        }

        _prevMeas = meas;

        float out = pTerm + _iTerm + dTerm;
        out = clamp(out,_outMin,_outMax);

        res.out = out;
        res.p   = pTerm;
        res.i   = _iTerm;
        res.d   = dTerm;
        return res;
    }

private:
    static float clamp(float v,float lo,float hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    float _kp,_ki,_kd;
    float _dt;
    float _iTerm;
    float _prevMeas;
    float _outMin,_outMax;
    float _iMin,_iMax;
    fc::filter::PT1Filter _dLpf;
};

} // namespace ctl
} // namespace fc
