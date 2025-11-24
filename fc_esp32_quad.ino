#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>

#include "platform_config.h"
#include "shared_types.h"
#include "imu_driver.h"
#include "attitude_estimator.h"
#include "pid_controller.h"
#include "motor_mixer.h"
#include "esc_driver.h"
#include "rc_receiver.h"
#include "rc_input.h"
#include "flight_mode_manager.h"
#include "motor_output_safety.h"
#include "failsafe_manager.h"
#include "shared_bus.h"
#include "tasking.h"
#include "flight_control_loop.h"
#include "auxiliary_services.h"

// === Feature flags PID tuner ===
// UDP PID tuner (dùng với Gradio)
#define PID_TUNE_UDP 1
#define PID_TUNE_UDP_WIFI_STA 0   // 0 = ESP phát AP, 1 = join router/hotspot

#define PID_TUNE_WIFI_SSID "ESP32_PID_TUNER"
#define PID_TUNE_WIFI_PASS "12345678"

#include "pid_tune_udp.h"   // module UDP PID tuner

#include "idle_finder.h"

// 0 = bay bình thường, 1 = chạy tool tìm idle
#define IDLE_FINDER_MODE 0

#define ESC_CALIBRATION_MODE 0   // Đổi thành 1 khi muốn calibrate ESC

using namespace fc;

// ==== Global objects ====

sync::SharedBus           g_bus;
imu::Mpu6050Driver        g_imu;
est::ComplementaryFilter  g_est;

// PID rate controllers
ctl::PidController g_pidRollRate;
ctl::PidController g_pidPitchRate;
ctl::PidController g_pidYawRate;

// Mixer
ctl::MotorMixerQuadX g_mixer;

// ESC
esc::EscPwm g_esc;

// RC
std::array<int,6> g_rcPins = {
    cfg::RC_CH1_PIN,
    cfg::RC_CH2_PIN,
    cfg::RC_CH3_PIN,
    cfg::RC_CH4_PIN,
    cfg::RC_CH5_PIN,
    cfg::RC_CH6_PIN
};
rc::PwmReceiver g_rcRx(g_rcPins);
rc::RcInput     g_rcInput(g_rcRx);

// Modes
modes::FlightModeManager g_modeMgr;

// Safety
safety::MotorOutputSafety g_motorSafety;
safety::FailsafeManager   g_failsafe(g_rcRx);

// GPS / Aux
TinyGPSPlus             g_gps;
loop::AuxiliaryServices g_aux(g_bus, Serial2, &g_gps);

// Control loop
loop::FlightControlLoop* g_fcLoopPtr = nullptr;

// ===== Tasks =====
class FlightControlTask : public tasking::ITask {
public:
    void run() override {
        if (g_fcLoopPtr) {
            g_fcLoopPtr->step(cfg::LOOP_DT_S);
        }
    }
};

class IoTask : public tasking::ITask {
public:
    void run() override {
        static uint32_t lastPrintMs = 0;
        g_aux.step(0.01f); // ~100Hz

        uint32_t now = millis();
        if (now - lastPrintMs > 100) { // 10Hz
            lastPrintMs = now;
            #if PID_TUNE_UDP
                float kp, ki, kd;
                fc::pidtune_udp::getCurrentPid(kp, ki, kd);
                Serial.print(F("PID: "));
                Serial.print(F("Kp=")); Serial.print(kp, 4);
                Serial.print(F(" Ki=")); Serial.print(ki, 4);
                Serial.print(F(" Kd=")); Serial.println(kd, 4);
            #endif
        }
    }
};

static FlightControlTask g_fcTask;
static IoTask            g_ioTask;

// TaskRunners
static tasking::TaskRunner g_fcRunner(
    g_fcTask,
    "flight_ctrl",
    18,          // priority
    4096,        // stack words
    0,           // core 0
    static_cast<uint32_t>(1000.0f / cfg::LOOP_HZ) // period ms
);

static tasking::TaskRunner g_ioRunner(
    g_ioTask,
    "io_task",
    10,          // priority trung bình
    4096,
    1,           // core 1
    10           // 10ms -> 100Hz
);

// ===== Setup PID defaults =====
void setupPidControllers() {
    float dt = cfg::LOOP_DT_S;

    g_pidRollRate.setDt(dt);
    g_pidPitchRate.setDt(dt);
    g_pidYawRate.setDt(dt);

    // Tạm thời đặt gain tương đối nhẹ; cần tune thực tế.
    g_pidRollRate.setGains(0.12f, 0.04f, 0.002f);
    g_pidPitchRate.setGains(0.12f, 0.04f, 0.002f);
    g_pidYawRate.setGains(0.20f, 0.05f, 0.0f);

    g_pidRollRate.setOutputLimits(-1.0f, 1.0f);
    g_pidPitchRate.setOutputLimits(-1.0f, 1.0f);
    g_pidYawRate.setOutputLimits(-1.0f, 1.0f);

    g_pidRollRate.setIntegralLimits(-0.5f, 0.5f);
    g_pidPitchRate.setIntegralLimits(-0.5f, 0.5f);
    g_pidYawRate.setIntegralLimits(-0.5f, 0.5f);

    g_pidRollRate.setDtermLpf(60.0f);
    g_pidPitchRate.setDtermLpf(60.0f);
    g_pidYawRate.setDtermLpf(30.0f);

    g_modeMgr.setAnglePGains(4.0f, 4.0f);
}

// ===== Arduino setup/loop =====

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println(F("===== FC ESP32 QUAD - INIT ====="));
    Serial.println(F("Pin Map: M1=13, M2=4, M3=14, M4=27; RC PWM CH1=34,CH2=35,CH3=32,CH4=33,CH5=25,CH6=26"));

    Serial.println(F("[IMU] Initializing MPU6050..."));
    if (!g_imu.begin()) {
        Serial.println(F("[IMU] ERROR: MPU6050 init failed."));
    } else {
        Serial.println(F("[IMU] OK."));

        // ==== IMU CALIBRATION ====
        Serial.println(F("[IMU] Calibrating... đặt drone nằm yên trên mặt phẳng!"));
        delay(2000); // cho drone ổn định 2s

        if (!g_imu.runCalibration(1000)) {   // lấy 1000 mẫu để tính offset
            Serial.println(F("[IMU] ERROR: calibration failed."));
        } else {
            Serial.println(F("[IMU] Calibration done."));
        }
    }


    // Init attitude estimator
    g_est.setDt(cfg::LOOP_DT_S);
    g_est.setTimeConstant(0.5f);
    g_est.setAccelLpfAlpha(0.8f);
    g_est.reset(0.0f, 0.0f, 0.0f);

    // Init ESC
    Serial.println(F("[ESC] Initializing ESC PWM..."));
    if (!g_esc.begin()) {
        Serial.println(F("[ESC] ERROR: ESC PWM init failed."));
    } else {
        Serial.println(F("[ESC] OK."));
    }
    g_esc.disarm();

    #if ESC_CALIBRATION_MODE
        Serial.println(F("*** ESC CALIBRATION MODE ENABLED ***"));
        Serial.println(F("THÁO TOÀN BỘ CÁNH TRƯỚC KHI TIẾP TỤC!!!"));
        delay(3000); // cho bạn thời gian đọc cảnh báo

        {
            fc::esc::EscCalibrator calib;
            calib.runCalibration(g_esc, 6000, 6000);
        }

        Serial.println(F("ESC calibration hoàn tất."));
        Serial.println(F("Hãy tắt nguồn (ngắt pin), đợi 5s,"));
        Serial.println(F("sau đó đặt ESC_CALIBRATION_MODE = 0 và upload lại code để bay bình thường."));

        // Dừng tại đây, không start task nào cả
        while (true) {
            vTaskDelay(portMAX_DELAY);
        }
    #endif

    // Init RC
    Serial.println(F("[RC] Initializing PWM receiver (dummy)..."));
    g_rcRx.begin();
    g_rcInput.setDeadband(0.03f);
    g_rcInput.setExpo(0.3f, 0.2f);
    g_rcInput.setSlewLimit(2.0f);

    #if IDLE_FINDER_MODE
        // Chỉ chạy tool tìm idle, KHÔNG start flight control loop.
        fc::calib::runIdleFinder(g_esc, g_rcInput);
        // runIdleFinder() khong bao gio return.
    #endif

    // Init GPS UART2
    Serial.println(F("[GPS] Init Serial2..."));
    Serial2.begin(cfg::GPS_BAUD, SERIAL_8N1, cfg::GPS_RX_PIN, cfg::GPS_TX_PIN);

    // Setup PID controllers
    setupPidControllers();

    // Motor safety
    g_motorSafety.setMinThrottleForSpin(0.02f);
    g_motorSafety.enableSoftRamp(true);
    g_motorSafety.setSoftRampRate(1.0f);

    // Mixer
    g_mixer.setMinThrottleForSpin(0.0f);

    // Construct flight control loop object
    static loop::FlightControlLoop fcLoop(
        g_imu,
        g_est,
        g_pidRollRate,
        g_pidPitchRate,
        g_pidYawRate,
        g_mixer,
        g_esc,
        g_rcInput,
        g_modeMgr,
        g_motorSafety,
        g_failsafe,
        g_bus
    );
    g_fcLoopPtr = &fcLoop;

    #if PID_TUNE_UDP
        // Dùng 0.0.0.0 => trong pid_tune_udp.h sẽ tự hiểu là broadcast
        // => Telemetry ATT gửi tới 255.255.255.255:14550
        // => Laptop nào trong mạng cũng nhận được, không cần biết IP trước
        IPAddress groundIp(0, 0, 0, 0);
        fc::pidtune_udp::init(
            PID_TUNE_WIFI_SSID,
            PID_TUNE_WIFI_PASS,
            groundIp,
            &g_bus,
            &g_pidRollRate,
            &g_pidPitchRate,
            &g_pidYawRate
        );
    #endif

    // Start FreeRTOS tasks
    g_fcRunner.start();
    g_ioRunner.start();

    Serial.println(F("===== FC ESP32 QUAD - READY ====="));
}

void loop() {
    #if PID_TUNE_UDP
        // xử lý UDP PID tuner (gửi ATT + nhận PID) trong loopTask
        fc::pidtune_udp::update();
    #endif

    // nhường CPU cho các task khác (WiFi, flight-control, v.v.)
    vTaskDelay(1);   // hoặc delay(1);
}
