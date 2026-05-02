#pragma once

#include <Arduino.h>
#include <math.h>
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

// 1=bật tuned update khi armed nhưng vẫn phải qua safety guards bên dưới.
#ifndef PID_TUNE_ALLOW_WHEN_ARMED
  #define PID_TUNE_ALLOW_WHEN_ARMED 0
#endif
#ifndef PID_TUNE_ARMED_REQUIRE_MODE
  #define PID_TUNE_ARMED_REQUIRE_MODE 255u
#endif
#ifndef PID_TUNE_ARMED_MIN_THROTTLE
  #define PID_TUNE_ARMED_MIN_THROTTLE 0.18f
#endif
#ifndef PID_TUNE_ARMED_MAX_THROTTLE
  #define PID_TUNE_ARMED_MAX_THROTTLE 0.65f
#endif
#ifndef PID_TUNE_ARMED_MAX_STICK
  #define PID_TUNE_ARMED_MAX_STICK 0.12f
#endif
#ifndef PID_TUNE_ARMED_APPLY_COOLDOWN_MS
  #define PID_TUNE_ARMED_APPLY_COOLDOWN_MS 1500u
#endif
#ifndef PID_TUNE_MAX_STEP_KP
  #define PID_TUNE_MAX_STEP_KP 0.030f
#endif
#ifndef PID_TUNE_MAX_STEP_KI
  #define PID_TUNE_MAX_STEP_KI 0.015f
#endif
#ifndef PID_TUNE_MAX_STEP_KD
  #define PID_TUNE_MAX_STEP_KD 0.005f
#endif
#ifndef PID_TUNE_MIN_KP
  #define PID_TUNE_MIN_KP 0.01f
#endif
#ifndef PID_TUNE_MAX_KP
  #define PID_TUNE_MAX_KP 0.80f
#endif
#ifndef PID_TUNE_MIN_KI
  #define PID_TUNE_MIN_KI 0.00f
#endif
#ifndef PID_TUNE_MAX_KI
  #define PID_TUNE_MAX_KI 0.30f
#endif
#ifndef PID_TUNE_MIN_KD
  #define PID_TUNE_MIN_KD 0.00f
#endif
#ifndef PID_TUNE_MAX_KD
  #define PID_TUNE_MAX_KD 0.08f
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
static float s_rollPitchKp = 0.12f, s_rollPitchKi = 0.04f, s_rollPitchKd = 0.002f;
static float s_yawKp = 0.20f, s_yawKi = 0.05f, s_yawKd = 0.0f;
static uint32_t s_seq = 0;
static uint32_t s_lastApplyMs = 0;

inline float clampf(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

inline void sanitizePid(float& kp, float& ki, float& kd) {
  kp = clampf(kp, PID_TUNE_MIN_KP, PID_TUNE_MAX_KP);
  ki = clampf(ki, PID_TUNE_MIN_KI, PID_TUNE_MAX_KI);
  kd = clampf(kd, PID_TUNE_MIN_KD, PID_TUNE_MAX_KD);
}

inline void getCurrentPid(float& rollPitchKp, float& rollPitchKi, float& rollPitchKd) {
  rollPitchKp = s_rollPitchKp;
  rollPitchKi = s_rollPitchKi;
  rollPitchKd = s_rollPitchKd;
}
inline void getCurrentPid(float& rollPitchKp, float& rollPitchKi, float& rollPitchKd,
                          float& yawKp, float& yawKi, float& yawKd) {
  rollPitchKp = s_rollPitchKp;
  rollPitchKi = s_rollPitchKi;
  rollPitchKd = s_rollPitchKd;
  yawKp = s_yawKp;
  yawKi = s_yawKi;
  yawKd = s_yawKd;
}
inline void applyPidToControllers() {
  if (s_rollPid) s_rollPid->setGains(s_rollPitchKp, s_rollPitchKi, s_rollPitchKd);
  if (s_pitchPid) s_pitchPid->setGains(s_rollPitchKp, s_rollPitchKi, s_rollPitchKd);
  if (s_yawPid) s_yawPid->setGains(s_yawKp, s_yawKi, s_yawKd);
}

inline IPAddress telemetryDest() {
  return (s_groundIp == IPAddress(0, 0, 0, 0)) ? IPAddress(255, 255, 255, 255) : s_groundIp;
}

inline void sendAck(const char* cmd, const char* status, const char* detail = nullptr) {
  char msg[96];
  if (detail && detail[0]) {
    snprintf(msg, sizeof(msg), "ACK,%s,%s,%s\n", cmd, status, detail);
  } else {
    snprintf(msg, sizeof(msg), "ACK,%s,%s\n", cmd, status);
  }
  IPAddress dest = telemetryDest();
  s_udp.beginPacket(dest, PID_TUNE_UDP_PORT);
  s_udp.write((const uint8_t*)msg, strlen(msg));
  s_udp.endPacket();
}

inline bool isStickCentered(const fc::types::ControlInput& rc) {
  return fabsf(rc.roll) <= PID_TUNE_ARMED_MAX_STICK &&
         fabsf(rc.pitch) <= PID_TUNE_ARMED_MAX_STICK &&
         fabsf(rc.yaw) <= PID_TUNE_ARMED_MAX_STICK;
}

inline bool isModeAllowed(const fc::types::ControlInput& rc) {
#if PID_TUNE_ARMED_REQUIRE_MODE <= 254u
  return rc.mode == PID_TUNE_ARMED_REQUIRE_MODE;
#else
  (void)rc;
  return true;
#endif
}

inline bool canApplyPidNow(const fc::types::ControlInput& rc, const char*& reason) {
  if (!rc.arm) {
    return true;
  }
#if PID_TUNE_ALLOW_WHEN_ARMED
  if (!isModeAllowed(rc)) {
    reason = "MODE";
    return false;
  }
  if (rc.throttle < PID_TUNE_ARMED_MIN_THROTTLE || rc.throttle > PID_TUNE_ARMED_MAX_THROTTLE) {
    reason = "THROTTLE";
    return false;
  }
  if (!isStickCentered(rc)) {
    reason = "STICKS";
    return false;
  }
  const uint32_t now = millis();
  if (now - s_lastApplyMs < PID_TUNE_ARMED_APPLY_COOLDOWN_MS) {
    reason = "COOLDOWN";
    return false;
  }
  return true;
#else
  reason = "ARMED_DISABLED";
  return false;
#endif
}

inline bool getTuneWindowStatus(const fc::types::ControlInput& rc, const char*& reason) {
  reason = nullptr;
  return canApplyPidNow(rc, reason);
}

inline bool validatePidStep(float currentKp, float currentKi, float currentKd,
                            float nextKp, float nextKi, float nextKd,
                            const char*& reason) {
  if (fabsf(nextKp - currentKp) > PID_TUNE_MAX_STEP_KP) {
    reason = "STEP_KP";
    return false;
  }
  if (fabsf(nextKi - currentKi) > PID_TUNE_MAX_STEP_KI) {
    reason = "STEP_KI";
    return false;
  }
  if (fabsf(nextKd - currentKd) > PID_TUNE_MAX_STEP_KD) {
    reason = "STEP_KD";
    return false;
  }
  return true;
}

inline bool parsePidTriple(const char* buf, const char* prefix, float& kp, float& ki, float& kd) {
  char fmt[16];
  snprintf(fmt, sizeof(fmt), "%s%%f,%%f,%%f", prefix);
  return sscanf(buf, fmt, &kp, &ki, &kd) == 3;
}

inline bool canApplyCommand(const char* cmd, const fc::types::ControlInput& rc,
                            float currentKp, float currentKi, float currentKd,
                            float nextKp, float nextKi, float nextKd) {
  const char* reason = nullptr;
  if (!validatePidStep(currentKp, currentKi, currentKd, nextKp, nextKi, nextKd, reason)) {
    sendAck(cmd, "REJECT", reason);
    return false;
  }
  if (!canApplyPidNow(rc, reason)) {
    sendAck(cmd, "REJECT", reason ? reason : "BLOCKED");
    return false;
  }
  return true;
}

inline void applyCommandSourceIp() {
  IPAddress remote = s_udp.remoteIP();
  if (remote != IPAddress(0, 0, 0, 0)) {
    s_groundIp = remote;
  }
}

inline void recvCommands() {
  int len = s_udp.parsePacket();
  if (len <= 0) return;
  char buf[96];
  if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
  len = s_udp.read(buf, len);
  buf[len] = '\0';
  applyCommandSourceIp();

  const char* cmd = "PID";
  if (strncmp(buf, "PIDRP,", 6) == 0) {
    cmd = "PIDRP";
  } else if (strncmp(buf, "PIDY,", 5) == 0) {
    cmd = "PIDY";
  }

  if (!s_bus) {
    sendAck(cmd, "REJECT", "NO_BUS");
    return;
  }

  fc::types::ControlInput rc{};
  if (!s_bus->getControlInput(rc)) {
    sendAck(cmd, "REJECT", "NO_RC");
    return;
  }

  if (strncmp(buf, "PIDRP,", 6) == 0) {
    float kp, ki, kd;
    if (!parsePidTriple(buf, "PIDRP,", kp, ki, kd)) {
      sendAck("PIDRP", "REJECT", "PARSE");
      return;
    }
    sanitizePid(kp, ki, kd);
    if (!canApplyCommand("PIDRP", rc, s_rollPitchKp, s_rollPitchKi, s_rollPitchKd, kp, ki, kd)) {
      return;
    }
    s_rollPitchKp = kp;
    s_rollPitchKi = ki;
    s_rollPitchKd = kd;
    applyPidToControllers();
    if (rc.arm) s_lastApplyMs = millis();
    sendAck("PIDRP", "OK");
    Serial.printf("[PID_UDP] RP PID: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
    return;
  }

  if (strncmp(buf, "PIDY,", 5) == 0) {
    float kp, ki, kd;
    if (!parsePidTriple(buf, "PIDY,", kp, ki, kd)) {
      sendAck("PIDY", "REJECT", "PARSE");
      return;
    }
    sanitizePid(kp, ki, kd);
    if (!canApplyCommand("PIDY", rc, s_yawKp, s_yawKi, s_yawKd, kp, ki, kd)) {
      return;
    }
    s_yawKp = kp;
    s_yawKi = ki;
    s_yawKd = kd;
    applyPidToControllers();
    if (rc.arm) s_lastApplyMs = millis();
    sendAck("PIDY", "OK");
    Serial.printf("[PID_UDP] Yaw PID: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
    return;
  }

  if (strncmp(buf, "PID,", 4) == 0) {
    float kp, ki, kd;
    if (!parsePidTriple(buf, "PID,", kp, ki, kd)) {
      sendAck("PID", "REJECT", "PARSE");
      return;
    }
    sanitizePid(kp, ki, kd);
    const char* reason = nullptr;
    if (!validatePidStep(s_rollPitchKp, s_rollPitchKi, s_rollPitchKd, kp, ki, kd, reason) ||
        !validatePidStep(s_yawKp, s_yawKi, s_yawKd, kp, ki, kd, reason)) {
      sendAck("PID", "REJECT", reason ? reason : "STEP");
      return;
    }
    if (!canApplyPidNow(rc, reason)) {
      sendAck("PID", "REJECT", reason ? reason : "BLOCKED");
      return;
    }
    s_rollPitchKp = kp;
    s_rollPitchKi = ki;
    s_rollPitchKd = kd;
    s_yawKp = kp;
    s_yawKi = ki;
    s_yawKd = kd;
    applyPidToControllers();
    if (rc.arm) s_lastApplyMs = millis();
    sendAck("PID", "OK");
    Serial.printf("[PID_UDP] All PID: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
    return;
  }

  sendAck("PID", "REJECT", "UNKNOWN_CMD");
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
  fc::types::RateTelemetry rate{};
  bool hasRate = s_bus->getRateTelemetry(rate);
  const char* reason = nullptr;
  bool tuneAllowed = hasRc && canApplyPidNow(rc, reason);
  IPAddress dest = telemetryDest();
  uint32_t nowMs = millis();
  uint32_t seq = ++s_seq;

#if PID_TUNE_TELEM_MODE == 0 || PID_TUNE_TELEM_MODE == 2
  char attbuf[160];
  int n1 = snprintf(attbuf, sizeof(attbuf),
                    "ATT,%lu,%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n",
                    (unsigned long)seq, (unsigned long)nowMs,
                    att.roll_deg, att.pitch_deg, att.yaw_deg,
                    s_rollPitchKp, s_rollPitchKi, s_rollPitchKd);
  s_udp.beginPacket(dest, PID_TUNE_UDP_PORT);
  s_udp.write((const uint8_t*)attbuf, n1);
  s_udp.endPacket();
#endif

#if PID_TUNE_TELEM_MODE == 1 || PID_TUNE_TELEM_MODE == 2
  char ext[320];
  int n2 = snprintf(ext, sizeof(ext),
                    "ATTX,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%d,%u,%d\n",
                    (unsigned long)seq, (unsigned long)nowMs,
                    att.roll_deg, att.pitch_deg, att.yaw_deg,
                    hasRate ? rate.roll_rate_dps : 0.0f,
                    hasRate ? rate.pitch_rate_dps : 0.0f,
                    hasRate ? rate.yaw_rate_dps : 0.0f,
                    hasRate ? rate.roll_setpoint_dps : 0.0f,
                    hasRate ? rate.pitch_setpoint_dps : 0.0f,
                    hasRate ? rate.yaw_setpoint_dps : 0.0f,
                    s_rollPitchKp, s_rollPitchKi, s_rollPitchKd,
                    s_yawKp, s_yawKi, s_yawKd,
                    hasRc ? rc.throttle : 0.0f,
                    hasRc ? (rc.arm ? 1 : 0) : 0,
                    hasRc ? static_cast<unsigned>(rc.mode) : 0u,
                    tuneAllowed ? 1 : 0);
  s_udp.beginPacket(dest, PID_TUNE_UDP_PORT);
  s_udp.write((const uint8_t*)ext, n2);
  s_udp.endPacket();
#endif
}

inline void init(const char* ssid, const char* password, IPAddress groundIp,
                 SharedBus* bus, PidController* rollPid, PidController* pitchPid, PidController* yawPid) {
  s_bus = bus; s_rollPid = rollPid; s_pitchPid = pitchPid; s_yawPid = yawPid; s_groundIp = groundIp;
  if (s_rollPid) {
    s_rollPid->getGains(s_rollPitchKp, s_rollPitchKi, s_rollPitchKd);
  } else if (s_pitchPid) {
    s_pitchPid->getGains(s_rollPitchKp, s_rollPitchKi, s_rollPitchKd);
  }
  if (s_yawPid) {
    s_yawPid->getGains(s_yawKp, s_yawKi, s_yawKd);
  }
  sanitizePid(s_rollPitchKp, s_rollPitchKi, s_rollPitchKd);
  sanitizePid(s_yawKp, s_yawKi, s_yawKd);
  s_lastApplyMs = 0;
  applyPidToControllers();
#if PID_TUNE_UDP_WIFI_STA
  const char* staSsid = ssid ? ssid : "";
  const char* staPass = password ? password : "";
  WiFi.mode(WIFI_STA); WiFi.begin(staSsid, staPass);
  int retries = 0; while (WiFi.status() != WL_CONNECTED && retries < 40) { delay(250); ++retries; }
  if (WiFi.status() != WL_CONNECTED) return;
  IPAddress ip = WiFi.localIP();
  Serial.printf("[PID_UDP] STA ssid=%s ip=%u.%u.%u.%u port=%u\n",
                staSsid,
                static_cast<unsigned>(ip[0]), static_cast<unsigned>(ip[1]),
                static_cast<unsigned>(ip[2]), static_cast<unsigned>(ip[3]),
                static_cast<unsigned>(PID_TUNE_UDP_PORT));
#else
  const char* apSsid = (ssid && ssid[0]) ? ssid : "ESP32_PID_TUNER";
  const char* apPass = (password && password[0]) ? password : "12345678";
  WiFi.mode(WIFI_AP); WiFi.softAP(apSsid, apPass);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[PID_UDP] AP ssid=%s ip=%u.%u.%u.%u port=%u\n",
                apSsid,
                static_cast<unsigned>(ip[0]), static_cast<unsigned>(ip[1]),
                static_cast<unsigned>(ip[2]), static_cast<unsigned>(ip[3]),
                static_cast<unsigned>(PID_TUNE_UDP_PORT));
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
inline void getCurrentPid(float& rollPitchKp, float& rollPitchKi, float& rollPitchKd,
                          float& yawKp, float& yawKi, float& yawKd) {
  rollPitchKp = rollPitchKi = rollPitchKd = 0.0f;
  yawKp = yawKi = yawKd = 0.0f;
}
inline bool getTuneWindowStatus(const fc::types::ControlInput&, const char*& reason) {
  reason = "OFF";
  return false;
}
}}
#endif
