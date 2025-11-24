#pragma once

#include <Arduino.h>

#include "platform_config.h"
#include "esc_driver.h"
#include "rc_input.h"
#include "shared_types.h"

namespace fc {
namespace calib {

// Mode bench nhỏ để tìm idle throttle cho combo ESC + motor.
// Cực kỳ lưu ý: THÁO CÁNH TRƯỚC KHI DÙNG.
inline void runIdleFinder(fc::esc::IEscDriver& esc,
                          fc::rc::RcInput&    rcInput)
{
    Serial.println();
    Serial.println(F("=== IDLE FINDER MODE ==="));
    Serial.println(F("!!! THAO TOAN BO CANH QUAT TRUOC KHI TEST !!!"));
    Serial.println(F("Huong dan:"));
    Serial.println(F("  1) Bat TX, RX nhu bay binh thuong."));
    Serial.println(F("  2) Giu ga ve 0, ARM tren tay dieu khien (CH5)."));
    Serial.println(F("  3) Nhe nha tang ga den muc motor quay deu, vua phai (idle mong muon)."));
    Serial.println(F("  4) Khi thay OK, day yaw stick HET BEN PHAI de CHOT idle."));
    Serial.println(F("  5) Doc gia tri de xuat tren Serial, copy vao g_motorSafety.setMinThrottleForSpin()."));
    Serial.println();

    esc.disarm();
    delay(500);

    Serial.println(F("[IDLE] Arm ESC o PWM_MIN..."));
    esc.arm();
    delay(1000);

    float    lastIdle    = 0.0f;
    uint32_t lastPrintMs = 0;

    while (true) {
        fc::types::ControlInput rc{};
        rcInput.getLatest(rc);  // da co deadband + expo

        // Điều khiển trực tiếp: 4 motor = throttle (0..1)
        fc::types::MotorCommand cmd{};
        if (rc.arm) {
            cmd.m1 = cmd.m2 = cmd.m3 = cmd.m4 = rc.throttle;
        } else {
            cmd.m1 = cmd.m2 = cmd.m3 = cmd.m4 = 0.0f;
        }
        esc.write(cmd);

        uint32_t nowMs = millis();
        if (nowMs - lastPrintMs > 200) {
            lastPrintMs = nowMs;
            Serial.print(F("[IDLE] thr="));
            Serial.print(rc.throttle, 3);
            Serial.print(F(" arm="));
            Serial.print(rc.arm ? 1 : 0);
            Serial.print(F(" yaw="));
            Serial.println(rc.yaw, 3);
        }

        // Khi ARM + yaw > 0.9 (stick het ben phai) => chốt idle hiện tại
        if (rc.arm && rc.yaw > 0.9f && rc.throttle > 0.02f) {
            lastIdle = rc.throttle;

            Serial.println();
            Serial.print(F("[IDLE] Ghi nhan idle throttle = "));
            Serial.println(lastIdle, 3);

            // Thêm chút margin để khi bay thật không bị hụt (motor dễ stop)
            float suggestedIdle = lastIdle + 0.02f;
            if (suggestedIdle > 0.30f) suggestedIdle = 0.30f;

            int pwmMin  = fc::cfg::PWM_MIN_US;
            int pwmMax  = fc::cfg::PWM_MAX_US;
            int span    = pwmMax - pwmMin;
            int pwmIdle = pwmMin + int(suggestedIdle * span + 0.5f);

            Serial.println(F("[IDLE] De xuat config:"));
            Serial.print  (F("    g_motorSafety.setMinThrottleForSpin("));
            Serial.print  (suggestedIdle, 3);
            Serial.println(F("f);"));
            Serial.print  (F("    // PWM idle ~ "));
            Serial.print  (pwmIdle);
            Serial.println(F(" us"));
            Serial.println();
            Serial.println(F("=> Chinh lai code, rebuild & flash de dung idle moi."));
            Serial.println();

            delay(1000);
        }

        delay(20); // ~50 Hz
    }
}

} // namespace calib
} // namespace fc
