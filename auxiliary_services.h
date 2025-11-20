
#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>

#include <Arduino.h>
#include "esp_timer.h"
#include <TinyGPSPlus.h>

#include "platform_config.h"
#include "shared_types.h"
#include "shared_bus.h"

namespace fc {
namespace loop {

class AuxiliaryServices {
public:
    AuxiliaryServices(fc::sync::SharedBus& bus,
                      HardwareSerial& gpsSerial,
                      TinyGPSPlus* gpsParser)
        : _bus(bus)
        , _gpsSerial(gpsSerial)
        , _gps(gpsParser)
        , _accumTime(0.0f)
        , _servicePeriod(0.02f)
    {
        pinMode(fc::cfg::VBAT_ADC_PIN, INPUT);
        analogReadResolution(12);
        analogSetPinAttenuation(fc::cfg::VBAT_ADC_PIN, ADC_11db);

        pinMode(fc::cfg::LED_PIN, OUTPUT);
        pinMode(fc::cfg::BUZZER_PIN, OUTPUT);
        digitalWrite(fc::cfg::LED_PIN, LOW);
        digitalWrite(fc::cfg::BUZZER_PIN, LOW);
    }

    void step(float dt_s) {
        if (dt_s <= 0.0f) return;
        _accumTime += dt_s;
        pollGpsSerial();
        if (_accumTime < _servicePeriod) return;
        _accumTime = 0.0f;

        updateBattery();
        updateGps();
        updateIndicators();
    }

private:
    void updateBattery() {
        int raw = analogRead(fc::cfg::VBAT_ADC_PIN);
        constexpr float VREF = 3.3f;
        float vAdc = (static_cast<float>(raw) / 4095.0f) * VREF;
        constexpr float R_TOP = 100000.0f;
        constexpr float R_BOTTOM = 22000.0f;
        float scale = (R_TOP + R_BOTTOM) / R_BOTTOM;
        float vbat = vAdc * scale;

        fc::types::BatteryState bs{};
        bs.vbat = vbat;
        bs.current = 0.0f;
        bs.stamp_us = static_cast<uint32_t>(esp_timer_get_time());
        _bus.setBatteryState(bs);
    }

    void pollGpsSerial() {
        if (_gps == nullptr) {
            while (_gpsSerial.available() > 0) {
                (void)_gpsSerial.read();
            }
            return;
        }
        while (_gpsSerial.available() > 0) {
            char c = static_cast<char>(_gpsSerial.read());
            _gps->encode(c);
        }
    }

    void updateGps() {
        if (_gps == nullptr) return;
        fc::types::GpsData gd{};
        gd.lat = _gps->location.isValid() ? _gps->location.lat() : 0.0;
        gd.lon = _gps->location.isValid() ? _gps->location.lng() : 0.0;
        gd.alt_m = _gps->altitude.isValid()
                  ? static_cast<float>(_gps->altitude.meters())
                  : 0.0f;
        gd.ground_speed_mps = _gps->speed.isValid()
                  ? static_cast<float>(_gps->speed.mps())
                  : 0.0f;
        gd.hdop = _gps->hdop.isValid()
                  ? static_cast<float>(_gps->hdop.hdop())
                  : 0.0f;
        gd.sats = _gps->satellites.isValid()
                  ? static_cast<uint8_t>(_gps->satellites.value())
                  : 0u;
        gd.fix = _gps->location.isValid() && _gps->location.isUpdated();
        gd.stamp_us = static_cast<uint32_t>(esp_timer_get_time());
        _bus.setGpsData(gd);
    }

    void updateIndicators() {
        fc::types::BatteryState bs{};
        bool haveBat = _bus.getBatteryState(bs);
        fc::types::ControlInput rc{};
        bool haveRc  = _bus.getControlInput(rc);

        bool lowBat = false;
        if (haveBat) {
            constexpr float VBAT_WARN_3S = 10.8f;
            lowBat = (bs.vbat > 0.1f && bs.vbat < VBAT_WARN_3S);
        }

        uint32_t now_ms = millis();
        if (lowBat) {
            bool on = ((now_ms / 250u) % 2u) == 0u;
            digitalWrite(fc::cfg::LED_PIN,    on ? HIGH : LOW);
            digitalWrite(fc::cfg::BUZZER_PIN, on ? HIGH : LOW);
        } else {
            if (haveRc && rc.arm) {
                digitalWrite(fc::cfg::LED_PIN, HIGH);
            } else {
                digitalWrite(fc::cfg::LED_PIN, LOW);
            }
            digitalWrite(fc::cfg::BUZZER_PIN, LOW);
        }
    }

    fc::sync::SharedBus& _bus;
    HardwareSerial&      _gpsSerial;
    TinyGPSPlus*         _gps;
    float _accumTime;
    float _servicePeriod;
};

} // namespace loop
} // namespace fc
