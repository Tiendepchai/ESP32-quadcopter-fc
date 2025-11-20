
#pragma once

#include <cstdint>
#include <cstddef>

#include "shared_types.h"

namespace fc {
namespace ctl {

class MotorMixerQuadX {
public:
    MotorMixerQuadX()
        : _minThrottleForSpin(0.0f)
    {}

    void setMinThrottleForSpin(float t) {
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        _minThrottleForSpin = t;
    }

    void compute(float throttle,
                 float rollCmd,
                 float pitchCmd,
                 float yawCmd,
                 fc::types::MotorCommand& out) const
    {
        float m1 = throttle + ( +rollCmd) + ( -pitchCmd) + ( -yawCmd);
        float m2 = throttle + ( +rollCmd) + ( +pitchCmd) + ( +yawCmd);
        float m3 = throttle + ( -rollCmd) + ( +pitchCmd) + ( -yawCmd);
        float m4 = throttle + ( -rollCmd) + ( -pitchCmd) + ( +yawCmd);

        if (_minThrottleForSpin > 0.0f && throttle < _minThrottleForSpin) {
            m1 = m2 = m3 = m4 = 0.0f;
        }

        out.m1 = clamp01(m1);
        out.m2 = clamp01(m2);
        out.m3 = clamp01(m3);
        out.m4 = clamp01(m4);
    }

private:
    static float clamp01(float v) {
        if (v < 0.0f) return 0.0f;
        if (v > 1.0f) return 1.0f;
        return v;
    }

    float _minThrottleForSpin;
};

} // namespace ctl
} // namespace fc
