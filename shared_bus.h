
#pragma once

#include <cstdint>
#include <cstddef>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "shared_types.h"

namespace fc {
namespace sync {

class SharedBus {
public:
    SharedBus() {
        _mux = portMUX_INITIALIZER_UNLOCKED;
        _hasAtt = _hasRc = _hasBat = _hasGps = false;
    }

    void setAttitude(const fc::types::FusedAttitude& att) {
        taskENTER_CRITICAL(&_mux);
        _att = att;
        _hasAtt = true;
        taskEXIT_CRITICAL(&_mux);
    }
    bool getAttitude(fc::types::FusedAttitude& out) const {
        taskENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
        bool h = _hasAtt;
        if (h) out = _att;
        taskEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
        return h;
    }

    void setControlInput(const fc::types::ControlInput& rc) {
        taskENTER_CRITICAL(&_mux);
        _rc = rc;
        _hasRc = true;
        taskEXIT_CRITICAL(&_mux);
    }
    bool getControlInput(fc::types::ControlInput& out) const {
        taskENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
        bool h = _hasRc;
        if (h) out = _rc;
        taskEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
        return h;
    }

    void setBatteryState(const fc::types::BatteryState& bs) {
        taskENTER_CRITICAL(&_mux);
        _bat = bs;
        _hasBat = true;
        taskEXIT_CRITICAL(&_mux);
    }
    bool getBatteryState(fc::types::BatteryState& out) const {
        taskENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
        bool h = _hasBat;
        if (h) out = _bat;
        taskEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
        return h;
    }

    void setGpsData(const fc::types::GpsData& gd) {
        taskENTER_CRITICAL(&_mux);
        _gps = gd;
        _hasGps = true;
        taskEXIT_CRITICAL(&_mux);
    }
    bool getGpsData(fc::types::GpsData& out) const {
        taskENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
        bool h = _hasGps;
        if (h) out = _gps;
        taskEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
        return h;
    }

private:
    mutable portMUX_TYPE _mux;

    fc::types::FusedAttitude _att;
    fc::types::ControlInput  _rc;
    fc::types::BatteryState  _bat;
    fc::types::GpsData       _gps;
    bool _hasAtt;
    bool _hasRc;
    bool _hasBat;
    bool _hasGps;
};

} // namespace sync
} // namespace fc
