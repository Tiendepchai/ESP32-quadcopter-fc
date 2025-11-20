
#pragma once

#include <cstdint>
#include <cstddef>

#include "shared_types.h"

namespace fc {
namespace modes {

enum class Mode {
    DISARM = 0,
    ANGLE,
    ACRO,
    ALTHOLD,
    POSHOLD,
    RTH
};

class FlightModeManager {
public:
    FlightModeManager()
        : _mode(Mode::DISARM)
        , _pAngleRoll(4.0f)
        , _pAnglePitch(4.0f)
    {}

    void setMode(Mode m) { _mode = m; }
    Mode getMode() const { return _mode; }

    void setAnglePGains(float pRoll,float pPitch) {
        _pAngleRoll  = pRoll;
        _pAnglePitch = pPitch;
    }

    void updateFromRc(const fc::types::ControlInput& rc) {
        if (!rc.arm) {
            _mode = Mode::DISARM;
            return;
        }
        switch (rc.mode) {
        case 0: _mode = Mode::ANGLE; break;
        case 1: _mode = Mode::ACRO;  break;
        default: _mode = Mode::ANGLE; break;
        }
    }

    void computeRateSetpoint(const fc::types::ControlInput& rc,
                             const fc::types::FusedAttitude& att,
                             float& desiredRateRoll,
                             float& desiredRatePitch,
                             float& desiredRateYaw) const
    {
        constexpr float maxAngleDeg             = 35.0f;
        constexpr float maxYawRateDegPerSec_Ang = 150.0f;
        constexpr float maxRateRollDegPerSec    = 400.0f;
        constexpr float maxRatePitchDegPerSec   = 400.0f;
        constexpr float maxRateYawDegPerSec     = 300.0f;

        desiredRateRoll  = 0.0f;
        desiredRatePitch = 0.0f;
        desiredRateYaw   = 0.0f;

        switch (_mode) {
        case Mode::DISARM:
            break;
        case Mode::ANGLE: {
            float angleSpRoll  = rc.roll  * maxAngleDeg;
            float angleSpPitch = rc.pitch * maxAngleDeg;
            float errRoll  = angleSpRoll  - att.roll_deg;
            float errPitch = angleSpPitch - att.pitch_deg;
            desiredRateRoll  = _pAngleRoll  * errRoll;
            desiredRatePitch = _pAnglePitch * errPitch;
            desiredRateYaw   = rc.yaw * maxYawRateDegPerSec_Ang;
            break;
        }
        case Mode::ACRO:
            desiredRateRoll  = rc.roll  * maxRateRollDegPerSec;
            desiredRatePitch = rc.pitch * maxRatePitchDegPerSec;
            desiredRateYaw   = rc.yaw   * maxRateYawDegPerSec;
            break;
        case Mode::ALTHOLD:
        case Mode::POSHOLD:
        case Mode::RTH:
        default: {
            float angleSpRoll  = rc.roll  * maxAngleDeg;
            float angleSpPitch = rc.pitch * maxAngleDeg;
            float errRoll  = angleSpRoll  - att.roll_deg;
            float errPitch = angleSpPitch - att.pitch_deg;
            desiredRateRoll  = _pAngleRoll  * errRoll;
            desiredRatePitch = _pAnglePitch * errPitch;
            desiredRateYaw   = rc.yaw * maxYawRateDegPerSec_Ang;
            break;
        }
        }
    }

private:
    Mode  _mode;
    float _pAngleRoll;
    float _pAnglePitch;
};

} // namespace modes
} // namespace fc
