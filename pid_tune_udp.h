#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "shared_bus.h"
#include "pid_controller.h"
#include "shared_types.h"

// Bật/tắt tính năng UDP PID tuner
#ifndef PID_TUNE_UDP
  #define PID_TUNE_UDP 0
#endif

// 0 = ESP làm AP phát WiFi, 1 = ESP join WiFi (STA)
#ifndef PID_TUNE_UDP_WIFI_STA
  #define PID_TUNE_UDP_WIFI_STA 0
#endif

// Cổng UDP dùng để giao tiếp với laptop
#ifndef PID_TUNE_UDP_PORT
  #define PID_TUNE_UDP_PORT 14550
#endif

// === NEW: cổng discovery (Python đang listen) ===
#ifndef PID_TUNE_UDP_DISCOVERY_PORT
  #define PID_TUNE_UDP_DISCOVERY_PORT 14555
#endif

// === NEW: tên board dùng trong gói DISC,... ===
#ifndef PID_TUNE_BOARD_NAME
  #define PID_TUNE_BOARD_NAME "FC_ESP32_QUAD"
#endif

#if PID_TUNE_UDP

namespace fc {
namespace pidtune_udp {

using fc::sync::SharedBus;
using fc::ctl::PidController;

// ====== state ======
static WiFiUDP      s_udp;
static IPAddress    s_groundIp;   // IP laptop / ground station (hoặc broadcast)
static SharedBus*   s_bus       = nullptr;
static PidController* s_rollPid  = nullptr;
static PidController* s_pitchPid = nullptr;
static PidController* s_yawPid   = nullptr;

// Mirror PID (dùng để gửi lên laptop, vì PidController ko có getGains)
static float s_kp = 0.12f;
static float s_ki = 0.04f;
static float s_kd = 0.002f;

// Cho code ngoài (IoTask) đọc PID hiện tại để in debug
inline void getCurrentPid(float &kp, float &ki, float &kd) {
    kp = s_kp;
    ki = s_ki;
    kd = s_kd;
}

// ===== helpers =====
inline void applyPidToControllers() {
    if (s_rollPid)  s_rollPid->setGains(s_kp, s_ki, s_kd);
    if (s_pitchPid) s_pitchPid->setGains(s_kp, s_ki, s_kd);
    if (s_yawPid)   s_yawPid->setGains(s_kp, s_ki, s_kd);
}

inline void recvCommands() {
    int len = s_udp.parsePacket();
    if (len <= 0) return;

    char buf[64];
    if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
    len = s_udp.read(buf, len);
    buf[len] = '\0';

    // Format: PID,kp,ki,kd\n
    if (strncmp(buf, "PID,", 4) == 0) {
        float kp, ki, kd;
        if (sscanf(buf, "PID,%f,%f,%f", &kp, &ki, &kd) == 3) {
            s_kp = kp;
            s_ki = ki;
            s_kd = kd;
            applyPidToControllers();
            Serial.printf("[PID_UDP] New PID: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
        }
    }
}

// === NEW: discovery broadcast ===
inline void sendDiscovery() {
    static uint32_t lastDiscMs = 0;
    uint32_t now = millis();
    if (now - lastDiscMs < 1000) { // mỗi ~1s
        return;
    }
    lastDiscMs = now;

    IPAddress bcast(255, 255, 255, 255);
    char msg[64];
    int n = snprintf(
        msg, sizeof(msg),
        "DISC,%s\n",
        PID_TUNE_BOARD_NAME
    );

    s_udp.beginPacket(bcast, PID_TUNE_UDP_DISCOVERY_PORT);
    s_udp.write((const uint8_t*)msg, n);
    s_udp.endPacket();
}

inline void sendTelemetry() {
    if (!s_bus) return;

    fc::types::FusedAttitude att{};
    if (!s_bus->getAttitude(att)) {
        return; // chưa có attitude
    }

    char buf[128];
    int n = snprintf(
        buf, sizeof(buf),
        "ATT,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n",
        att.roll_deg,
        att.pitch_deg,
        att.yaw_deg,
        s_kp, s_ki, s_kd
    );

    // Nếu s_groundIp là 0.0.0.0 thì coi như broadcast
    IPAddress destIp = s_groundIp;
    if (destIp == IPAddress(0, 0, 0, 0)) {
        destIp = IPAddress(255, 255, 255, 255);
    }

    s_udp.beginPacket(destIp, PID_TUNE_UDP_PORT);
    s_udp.write((const uint8_t*)buf, n);
    s_udp.endPacket();
}

// ===== API: init + update =====
inline void init(const char* ssid,
                 const char* password,
                 IPAddress   groundIp,
                 SharedBus*  bus,
                 PidController* rollPid,
                 PidController* pitchPid,
                 PidController* yawPid)
{
    s_bus      = bus;
    s_rollPid  = rollPid;
    s_pitchPid = pitchPid;
    s_yawPid   = yawPid;
    s_groundIp = groundIp;

    applyPidToControllers();

#if PID_TUNE_UDP_WIFI_STA
    // ESP join WiFi sẵn có (router / hotspot)
    const char* staSsid = ssid     ? ssid     : "";
    const char* staPass = password ? password : "";

    WiFi.mode(WIFI_STA);
    WiFi.begin(staSsid, staPass);
    Serial.printf("[PID_UDP] Connecting STA: %s\n", staSsid);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 40) {
        delay(250);
        Serial.print('.');
        ++retries;
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[PID_UDP] WiFi connect FAILED, UDP tuner disabled");
        return;
    }
    Serial.print("[PID_UDP] STA IP: ");
    Serial.println(WiFi.localIP());
#else
    // ESP làm AP phát WiFi
    const char* apSsid = (ssid && ssid[0]) ? ssid : "ESP32_PID_TUNER";
    const char* apPass = (password && password[0]) ? password : "12345678";

    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSsid, apPass);
    Serial.print("[PID_UDP] AP SSID: ");
    Serial.println(apSsid);
    Serial.print("[PID_UDP] AP IP: ");
    Serial.println(WiFi.softAPIP());
#endif

    // Start UDP
    if (s_udp.begin(PID_TUNE_UDP_PORT)) {
        Serial.printf("[PID_UDP] UDP listening on port %u\n", PID_TUNE_UDP_PORT);
        Serial.print("[PID_UDP] Ground IP (0.0.0.0=broadcast): ");
        Serial.println(s_groundIp);
    } else {
        Serial.println("[PID_UDP] Failed to start UDP");
    }
}

inline void update() {
    // Không block, gọi càng thường xuyên càng tốt (loop / IoTask)
    recvCommands();

    static uint32_t lastSendMs = 0;
    uint32_t now = millis();
    if (now - lastSendMs >= 20) { // ~50 Hz
        lastSendMs = now;
        sendTelemetry();
    }

    // === NEW: discovery mỗi ~1s ===
    sendDiscovery();
}

} // namespace pidtune_udp
} // namespace fc

#else  // PID_TUNE_UDP == 0

// Stub khi tắt tính năng để code khác vẫn compile được
namespace fc {
namespace pidtune_udp {
inline void init(const char*,
                 const char*,
                 IPAddress,
                 fc::sync::SharedBus*,
                 fc::ctl::PidController*,
                 fc::ctl::PidController*,
                 fc::ctl::PidController*) {}
inline void update() {}
} // namespace pidtune_udp
} // namespace fc

#endif // PID_TUNE_UDP
