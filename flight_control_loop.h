
#pragma once

#include <cstdint>
#include <cstddef>

#include "shared_types.h"
#include "imu_driver.h"
#include "attitude_estimator.h"
#include "pid_controller.h"
#include "motor_mixer.h"
#include "esc_driver.h"
#include "rc_input.h"
#include "flight_mode_manager.h"
#include "motor_output_safety.h"
#include "failsafe_manager.h"
#include "shared_bus.h"

namespace fc {
namespace loop {

class FlightControlLoop {
public:
    FlightControlLoop(fc::imu::IImuDriver&          imu,
                      fc::est::IAttitudeEstimator&  est,
                      fc::ctl::PidController&       pidRateRoll,
                      fc::ctl::PidController&       pidRatePitch,
                      fc::ctl::PidController&       pidRateYaw,
                      fc::ctl::MotorMixerQuadX&     mixer,
                      fc::esc::IEscDriver&          esc,
                      fc::rc::RcInput&              rcInput,
                      fc::modes::FlightModeManager& modeMgr,
                      fc::safety::MotorOutputSafety& motorSafety,
                      fc::safety::FailsafeManager&   failsafe,
                      fc::sync::SharedBus&           bus
    )
        : _imu(imu)
        , _est(est)
        , _pidRateRoll(pidRateRoll)
        , _pidRatePitch(pidRatePitch)
        , _pidRateYaw(pidRateYaw)
        , _mixer(mixer)
        , _esc(esc)
        , _rcInput(rcInput)
        , _modeMgr(modeMgr)
        , _motorSafety(motorSafety)
        , _failsafe(failsafe)
        , _bus(bus)
        , _escArmedState(false)   // NEW
    {}

    void step(float dt_s) {
        fc::types::ImuSample s{};
        if (!_imu.read(s)) {
            return;
        }

        _est.update(s, dt_s);
        fc::types::FusedAttitude att = _est.get();
        _bus.setAttitude(att);

        fc::types::ControlInput rc{};
        _rcInput.getLatest(rc);
        _bus.setControlInput(rc);

        _modeMgr.updateFromRc(rc);

        float desiredRateRoll  = 0.0f;
        float desiredRatePitch = 0.0f;
        float desiredRateYaw   = 0.0f;
        _modeMgr.computeRateSetpoint(rc, att,
                                     desiredRateRoll,
                                     desiredRatePitch,
                                     desiredRateYaw);

        float rate_meas_roll  = s.gx;
        float rate_meas_pitch = s.gy;
        float rate_meas_yaw   = s.gz;

        auto pidRoll  = _pidRateRoll .compute(rate_meas_roll,  desiredRateRoll);
        auto pidPitch = _pidRatePitch.compute(rate_meas_pitch, desiredRatePitch);
        auto pidYaw   = _pidRateYaw  .compute(rate_meas_yaw,   desiredRateYaw);

        fc::types::MotorCommand motors{};
        _mixer.compute(rc.throttle, pidRoll.out, pidPitch.out, pidYaw.out, motors);

        _failsafe.update();
        _failsafe.applyFailsafe(rc, motors);

        // Hệ thống được coi là "armed" khi RC arm và không failsafe
        bool systemArmed = rc.arm && !_failsafe.isActive();

        // Đồng bộ trạng thái này với MotorOutputSafety
        _motorSafety.setArmed(systemArmed);
        _motorSafety.applySafety(rc, motors);
        _motorSafety.applySoftRamp(rc, motors, dt_s);

        // Đồng bộ với ESC: nếu vừa arm → gọi esc.arm(), nếu disarm → esc.disarm()
        if (systemArmed && !_escArmedState) {
            _esc.arm();
            _escArmedState = true;
        } else if (!systemArmed && _escArmedState) {
            _esc.disarm();
            _escArmedState = false;
        }

        // Cuối cùng mới gửi lệnh
        _esc.write(motors);
    }

private:
    fc::imu::IImuDriver&            _imu;
    fc::est::IAttitudeEstimator&    _est;
    fc::ctl::PidController&         _pidRateRoll;
    fc::ctl::PidController&         _pidRatePitch;
    fc::ctl::PidController&         _pidRateYaw;
    fc::ctl::MotorMixerQuadX&       _mixer;
    fc::esc::IEscDriver&            _esc;
    fc::rc::RcInput&                _rcInput;
    fc::modes::FlightModeManager&   _modeMgr;
    fc::safety::MotorOutputSafety&  _motorSafety;
    fc::safety::FailsafeManager&    _failsafe;
    fc::sync::SharedBus&            _bus;
    bool                            _escArmedState;   // NEW
};

} // namespace loop
} // namespace fc
