#pragma once

#include "shared_bus.h"
#include "pid_controller.h"
#include "shared_types.h"

// Cho phép dùng cả PID_TUNE_WEBSOCKET hoặc PID_tune_websocket
#ifndef PID_TUNE_WEBSOCKET
  #ifdef PID_tune_websocket
    #define PID_TUNE_WEBSOCKET PID_tune_websocket
  #else
    #define PID_TUNE_WEBSOCKET 0
  #endif
#endif


#if PID_TUNE_WEBSOCKET

#include <WiFi.h>
#include <ArduinoWebsockets.h>

namespace fc {
namespace pidtune_ws {

using namespace websockets;

// ==== State nội bộ của module ====
static WiFiServer       s_httpServer(80);
static WebsocketsServer s_wsServer;
static WebsocketsClient s_wsClient;
static bool             s_wsConnected = false;

static sync::SharedBus*    s_bus      = nullptr;
static ctl::PidController* s_rollPid  = nullptr;
static ctl::PidController* s_pitchPid = nullptr;
static ctl::PidController* s_yawPid   = nullptr;

// PID hiện tại (hiển thị trên UI)
static float s_kp = 0.12f;
static float s_ki = 0.04f;
static float s_kd = 0.002f;

// Thông tin AP mặc định (có thể truyền ssid/pass khác khi init)
static const char* s_defaultSsid = "ESP32_PID_TUNER";
static const char* s_defaultPass = "12345678";

// ====== HTML giao diện + Chart.js ======
static const char PID_HTML[] PROGMEM = R"=====(<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>ESP32 PID Tuner</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; margin: 16px; background:#111; color:#eee; }
    h2, h3 { margin-top: 0; }
    .card { background:#222; padding:16px; border-radius:8px; margin-bottom:16px; }
    .row  { display:flex; align-items:center; margin-bottom:8px; }
    .row label { width:40px; }
    .row input[type=range] { flex:1; margin:0 8px; }
    .row span { width:70px; text-align:right; font-variant-numeric: tabular-nums; }
    #status { margin-bottom:8px; }
    canvas { width:100%; max-width:100%; height:260px; }
  </style>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
  <h2>ESP32 PID Tuner</h2>
  <div id="status">WebSocket: <span id="wsState">connecting...</span></div>

  <div class="card">
    <div class="row">
      <label for="kp">Kp</label>
      <input id="kp" type="range" min="0" max="1" step="0.005" value="0.12">
      <span id="kpVal">0.120</span>
    </div>
    <div class="row">
      <label for="ki">Ki</label>
      <input id="ki" type="range" min="0" max="0.5" step="0.001" value="0.04">
      <span id="kiVal">0.040</span>
    </div>
    <div class="row">
      <label for="kd">Kd</label>
      <input id="kd" type="range" min="0" max="0.05" step="0.0005" value="0.002">
      <span id="kdVal">0.002</span>
    </div>
  </div>

  <div class="card">
    <h3>Góc nghiêng (Roll / Pitch)</h3>
    <canvas id="attChart"></canvas>
  </div>

<script>
let ws;
let kpSlider, kiSlider, kdSlider;
let kpVal, kiVal, kdVal;
let chart;
const maxPoints = 150;

function connectWS() {
  const url = "ws://" + window.location.hostname + ":81/";
  ws = new WebSocket(url);

  ws.onopen = () => {
    document.getElementById('wsState').textContent = 'connected';
    sendPid();
  };

  ws.onclose = () => {
    document.getElementById('wsState').textContent = 'disconnected, retrying...';
    setTimeout(connectWS, 2000);
  };

  ws.onerror = () => {
    document.getElementById('wsState').textContent = 'error';
  };

  ws.onmessage = (event) => {
    const msg = event.data;
    if (msg.startsWith("ATT,")) {
      const parts = msg.split(",");
      const roll  = parseFloat(parts[1]);
      const pitch = parseFloat(parts[2]);
      addAttPoint(roll, pitch);
    }
  };
}

function sendPid() {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  const kp = parseFloat(kpSlider.value);
  const ki = parseFloat(kiSlider.value);
  const kd = parseFloat(kdSlider.value);
  const line = "PID," + kp.toFixed(4) + "," + ki.toFixed(4) + "," + kd.toFixed(4);
  ws.send(line);
}

function addAttPoint(roll, pitch) {
  const labels    = chart.data.labels;
  const rollData  = chart.data.datasets[0].data;
  const pitchData = chart.data.datasets[1].data;

  if (labels.length >= maxPoints) {
    labels.shift();
    rollData.shift();
    pitchData.shift();
  }
  labels.push("");
  rollData.push(roll);
  pitchData.push(pitch);
  chart.update();
}

function setupChart() {
  const ctx = document.getElementById('attChart').getContext('2d');
  chart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'Roll (deg)',  data: [], borderWidth: 1, fill: false },
        { label: 'Pitch (deg)', data: [], borderWidth: 1, fill: false }
      ]
    },
    options: {
      animation: false,
      scales: {
        y: { suggestedMin: -60, suggestedMax: 60 }
      },
      plugins: {
        legend: { display: true }
      }
    }
  });
}

window.addEventListener('load', () => {
  kpSlider = document.getElementById('kp');
  kiSlider = document.getElementById('ki');
  kdSlider = document.getElementById('kd');
  kpVal    = document.getElementById('kpVal');
  kiVal    = document.getElementById('kiVal');
  kdVal    = document.getElementById('kdVal');

  const onSlider = () => {
    kpVal.textContent = parseFloat(kpSlider.value).toFixed(3);
    kiVal.textContent = parseFloat(kiSlider.value).toFixed(3);
    kdVal.textContent = parseFloat(kdSlider.value).toFixed(3);
    sendPid();
  };

  kpSlider.addEventListener('input', onSlider);
  kiSlider.addEventListener('input', onSlider);
  kdSlider.addEventListener('input', onSlider);

  setupChart();
  connectWS();
});
</script>
</body>
</html>)=====";

// ===== Helpers nội bộ =====
inline void applyPid() {
    if (s_rollPid)  s_rollPid->setGains(s_kp, s_ki, s_kd);
    if (s_pitchPid) s_pitchPid->setGains(s_kp, s_ki, s_kd);
    // nếu cần: if (s_yawPid) s_yawPid->setGains(s_kp, s_ki, s_kd);
}

inline void handleHttp() {
    WiFiClient client = s_httpServer.available();
    if (!client) return;

    String req = client.readStringUntil('\r');
    client.readStringUntil('\n');

    while (client.available()) {
        String line = client.readStringUntil('\n');
        if (line == "\r" || line.length() == 0) break;
    }

    if (req.indexOf("GET / ") >= 0) {
        client.println(F("HTTP/1.1 200 OK"));
        client.println(F("Content-Type: text/html"));
        client.println(F("Connection: close"));
        client.println();
        client.print(PID_HTML);
    } else {
        client.println(F("HTTP/1.1 404 Not Found"));
        client.println(F("Connection: close"));
        client.println();
    }

    delay(1);
    client.stop();
}

inline void onMessage(WebsocketsMessage msg) {
    String data = msg.data();
    if (data.startsWith("PID,")) {
        float kp, ki, kd;
        if (sscanf(data.c_str(), "PID,%f,%f,%f", &kp, &ki, &kd) == 3) {
            s_kp = kp;
            s_ki = ki;
            s_kd = kd;
            applyPid();
            Serial.printf("[PID_WS] New PID: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
        }
    }
}

inline void onEvent(WebsocketsEvent event, String) {
    if (event == WebsocketsEvent::ConnectionOpened) {
        Serial.println("[PID_WS] Client connected");
    } else if (event == WebsocketsEvent::ConnectionClosed) {
        Serial.println("[PID_WS] Client disconnected");
        s_wsConnected = false;
    }
}

inline void handleWs() {
    if (!s_wsConnected) {
        auto client = s_wsServer.accept();
        if (client.available()) {
            s_wsClient    = client;
            s_wsConnected = true;
            s_wsClient.onMessage(onMessage);
            s_wsClient.onEvent(onEvent);
            Serial.println("[PID_WS] New client accepted");
        }
    }

    if (s_wsConnected) {
        s_wsClient.poll();

        static uint32_t lastSendMs = 0;
        uint32_t now = millis();
        if (now - lastSendMs >= 20) { // ~50 Hz
            lastSendMs = now;
            if (s_bus) {
                fc::types::FusedAttitude att{};
                if (s_bus->getAttitude(att)) {
                    char buf[64];
                    snprintf(buf, sizeof(buf),
                             "ATT,%.2f,%.2f",
                             att.roll_deg, att.pitch_deg);
                    s_wsClient.send(buf);
                }
            }
        }
    }
}

// ===== API public =====
inline void init(const char* ssid,
                 const char* password,
                 sync::SharedBus* bus,
                 ctl::PidController* rollPid,
                 ctl::PidController* pitchPid,
                 ctl::PidController* yawPid)
{
    s_bus      = bus;
    s_rollPid  = rollPid;
    s_pitchPid = pitchPid;
    s_yawPid   = yawPid;

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid ? ssid : s_defaultSsid,
                password ? password : s_defaultPass);
    IPAddress ip = WiFi.softAPIP();
    Serial.print("[PID_WS] AP IP: ");
    Serial.println(ip);

    s_httpServer.begin();
    s_wsServer.listen(81);

    Serial.println("[PID_WS] HTTP server on :80");
    Serial.println("[PID_WS] WS server   on :81");
}

inline void update() {
    handleHttp();
    handleWs();
}

} // namespace pidtune_ws
} // namespace fc

#else  // PID_TUNE_WEBSOCKET == 0

// Khi tắt flag, cung cấp stub để code khác vẫn compile được
namespace fc {
namespace pidtune_ws {
inline void init(const char*,
                 const char*,
                 sync::SharedBus*,
                 ctl::PidController*,
                 ctl::PidController*,
                 ctl::PidController*) {}
inline void update() {}
} // namespace pidtune_ws
} // namespace fc

#endif // PID_TUNE_WEBSOCKET
