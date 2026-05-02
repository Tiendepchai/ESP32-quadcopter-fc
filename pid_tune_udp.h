#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "shared_bus.h"
#include "pid_controller.h"
#include "shared_types.h"

#ifndef PID_TUNE_UDP
  #define PID_TUNE_UDP 0
#endif
#ifndef PID_TUNE_UDP_WIFI_STA
  #define PID_TUNE_UDP_WIFI_STA 0
#endif
#ifndef PID_TUNE_UDP_PORT
  #define PID_TUNE_UDP_PORT 14550
#endif
#ifndef PID_TUNE_UDP_DISCOVERY_PORT
  #define PID_TUNE_UDP_DISCOVERY_PORT 14555
#endif
#ifndef PID_TUNE_BOARD_NAME
  #define PID_TUNE_BOARD_NAME "FC_ESP32_QUAD"
#endif

// 0=ATT only, 1=ATTX only, 2=ATT+ATTX
#ifndef PID_TUNE_TELEM_MODE
  #define PID_TUNE_TELEM_MODE 1
#endif

// 1=chỉ cho phép set PID khi disarm, 0=cho phép luôn
#ifndef PID_TUNE_ALLOW_WHEN_ARMED
  #define PID_TUNE_ALLOW_WHEN_ARMED 0
#endif

#if PID_TUNE_UDP
namespace fc { namespace pidtune_udp {
using fc::sync::SharedBus;
using fc::ctl::PidController;

static WiFiUDP s_udp;
static IPAddress s_groundIp;
static SharedBus* s_bus = nullptr;
static PidController* s_rollPid = nullptr;
static PidController* s_pitchPid = nullptr;
static PidController* s_yawPid = nullptr;
static float s_kp = 0.12f, s_ki = 0.04f, s_kd = 0.002f;
static uint32_t s_seq = 0;

inline void getCurrentPid(float& kp, float& ki, float& kd) { kp = s_kp; ki = s_ki; kd = s_kd; }
inline void applyPidToControllers() {
  if (s_rollPid) s_rollPid->setGains(s_kp, s_ki, s_kd);
  if (s_pitchPid) s_pitchPid->setGains(s_kp, s_ki, s_kd);
  if (s_yawPid) s_yawPid->setGains(s_kp, s_ki, s_kd);
}

inline IPAddress telemetryDest() {
  return (s_groundIp == IPAddress(0, 0, 0, 0)) ? IPAddress(255, 255, 255, 255) : s_groundIp;
}

inline void sendAck(const char* msg) {
  IPAddress dest = telemetryDest();
  s_udp.beginPacket(dest, PID_TUNE_UDP_PORT);
  s_udp.write((const uint8_t*)msg, strlen(msg));
  s_udp.endPacket();
}

inline bool canApplyPidNow() {
#if PID_TUNE_ALLOW_WHEN_ARMED
  return true;
#else
  if (!s_bus) return false;
  fc::types::ControlInput rc{};
  if (!s_bus->getControlInput(rc)) return false;
  return !rc.arm;
#endif
}

inline void recvCommands() {
  int len = s_udp.parsePacket();
  if (len <= 0) return;
  char buf[96];
  if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
  len = s_udp.read(buf, len);
  buf[len] = '\0';

  if (strncmp(buf, "PID,", 4) == 0) {
    float kp, ki, kd;
    if (sscanf(buf, "PID,%f,%f,%f", &kp, &ki, &kd) == 3) {
      if (!canApplyPidNow()) {
        sendAck("ACK,PID,REJECT,ARMED\n");
        return;
      }
      s_kp = kp; s_ki = ki; s_kd = kd;
      applyPidToControllers();
      sendAck("ACK,PID,OK\n");
      Serial.printf("[PID_UDP] New PID: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
    }
  }
}

inline void sendDiscovery() {
  static uint32_t lastDiscMs = 0;
  uint32_t now = millis();
  if (now - lastDiscMs < 1000) return;
  lastDiscMs = now;
  IPAddress bcast(255, 255, 255, 255);
  char msg[96];
  int n = snprintf(msg, sizeof(msg), "DISC,%s,%u\n", PID_TUNE_BOARD_NAME, PID_TUNE_UDP_PORT);
  s_udp.beginPacket(bcast, PID_TUNE_UDP_DISCOVERY_PORT);
  s_udp.write((const uint8_t*)msg, n);
  s_udp.endPacket();
}

inline void sendTelemetry() {
  if (!s_bus) return;
  fc::types::FusedAttitude att{};
  if (!s_bus->getAttitude(att)) return;

  fc::types::ControlInput rc{};
  bool hasRc = s_bus->getControlInput(rc);
  IPAddress dest = telemetryDest();
  uint32_t nowMs = millis();
  uint32_t seq = ++s_seq;

#if PID_TUNE_TELEM_MODE == 0 || PID_TUNE_TELEM_MODE == 2
  char attbuf[160];
  int n1 = snprintf(attbuf, sizeof(attbuf),
                    "ATT,%lu,%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n",
                    (unsigned long)seq, (unsigned long)nowMs,
                    att.roll_deg, att.pitch_deg, att.yaw_deg,
                    s_kp, s_ki, s_kd);
  s_udp.beginPacket(dest, PID_TUNE_UDP_PORT);
  s_udp.write((const uint8_t*)attbuf, n1);
  s_udp.endPacket();
#endif

#if PID_TUNE_TELEM_MODE == 1 || PID_TUNE_TELEM_MODE == 2
  char ext[196];
  int n2 = snprintf(ext, sizeof(ext),
                    "ATTX,%lu,%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.3f,%d,%u\n",
                    (unsigned long)seq, (unsigned long)nowMs,
                    att.roll_deg, att.pitch_deg, att.yaw_deg,
                    s_kp, s_ki, s_kd,
                    hasRc ? rc.throttle : 0.0f,
                    hasRc ? (rc.arm ? 1 : 0) : 0,
                    hasRc ? static_cast<unsigned>(rc.mode) : 0u);
  s_udp.beginPacket(dest, PID_TUNE_UDP_PORT);
  s_udp.write((const uint8_t*)ext, n2);
  s_udp.endPacket();
#endif
}

inline void init(const char* ssid, const char* password, IPAddress groundIp,
                 SharedBus* bus, PidController* rollPid, PidController* pitchPid, PidController* yawPid) {
  s_bus = bus; s_rollPid = rollPid; s_pitchPid = pitchPid; s_yawPid = yawPid; s_groundIp = groundIp;
  applyPidToControllers();
#if PID_TUNE_UDP_WIFI_STA
  const char* staSsid = ssid ? ssid : "";
  const char* staPass = password ? password : "";
  WiFi.mode(WIFI_STA); WiFi.begin(staSsid, staPass);
  int retries = 0; while (WiFi.status() != WL_CONNECTED && retries < 40) { delay(250); ++retries; }
  if (WiFi.status() != WL_CONNECTED) return;
#else
  const char* apSsid = (ssid && ssid[0]) ? ssid : "ESP32_PID_TUNER";
  const char* apPass = (password && password[0]) ? password : "12345678";
  WiFi.mode(WIFI_AP); WiFi.softAP(apSsid, apPass);
#endif
  if (s_udp.begin(PID_TUNE_UDP_PORT)) {
    Serial.printf("[PID_UDP] UDP listening on port %u\n", PID_TUNE_UDP_PORT);
  }
}

inline void update() {
  recvCommands();
  static uint32_t lastSendMs = 0;
  uint32_t now = millis();
  if (now - lastSendMs >= 20) { lastSendMs = now; sendTelemetry(); }
  sendDiscovery();
}

}} // ns
#else
namespace fc { namespace pidtune_udp {
inline void init(const char*, const char*, IPAddress, fc::sync::SharedBus*, fc::ctl::PidController*, fc::ctl::PidController*, fc::ctl::PidController*) {}
inline void update() {}
inline void getCurrentPid(float& kp, float& ki, float& kd) { kp = ki = kd = 0.0f; }
}}
#endif
