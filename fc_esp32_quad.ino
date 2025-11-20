
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

// === Feature flag đặt ở đây ===
// 1 = bật Web PID tuner, 0 = tắt
#define PID_TUNE_WEBSOCKET 1
#define PID_TUNE_WIFI_SSID "ESP32_PID_TUNER"
#define PID_TUNE_WIFI_PASS "12345678"
#include "pid_tune_websocket.h"

using namespace fc;

#define ESC_CALIBRATION_MODE  0   // Đổi thành 1 khi muốn calibrate ESC

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
TinyGPSPlus            g_gps;
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
    // void run() override {
    //     g_aux.step(0.1f); // ~100Hz
    // }
    void run() override {
        static uint32_t lastPrintMs = 0;
        g_aux.step(0.01f); // ~100Hz

        #if PID_TUNE_WEBSOCKET
            // Xử lý HTTP + WebSocket (PID tuner + streaming attitude)
            pidtune_ws::update();
        #endif

        uint32_t now = millis();
        if (now - lastPrintMs > 100) { // 10Hz
            lastPrintMs = now;
            fc::types::FusedAttitude att{};
            fc::types::ControlInput rc{};

            if (g_bus.getAttitude(att)) {
                Serial.print(F("ATT: "));
                Serial.print(att.roll_deg);  Serial.print(F(" / "));
                Serial.print(att.pitch_deg); Serial.print(F(" / "));
                Serial.println(att.yaw_deg);
            }
            if (g_bus.getControlInput(rc)) {
                Serial.print(F("RC:  "));
                Serial.print(rc.roll);      Serial.print(F(" "));
                Serial.print(rc.pitch);     Serial.print(F(" "));
                Serial.print(rc.yaw);       Serial.print(F(" "));
                Serial.print(rc.throttle);  Serial.print(F(" arm="));
                Serial.print(rc.arm);       Serial.print(F(" mode="));
                Serial.println(rc.mode);
            }
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
    Serial.println(F("Pin Map: M1=13, M2=12, M3=14, M4=27; RC PWM CH1=34,CH2=35,CH3=32,CH4=33,CH5=25,CH6=26"));

    // Init I2C & IMU
    Serial.println(F("[IMU] Initializing MPU6050..."));
    if (!g_imu.begin()) {
        Serial.println(F("[IMU] ERROR: MPU6050 init failed."));
    } else {
        Serial.println(F("[IMU] OK."));
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
        Serial.println(F("THAO TOAN BO CANH TRUOC KHI TIEP TUC!!!"));
        delay(3000); // cho ban thoi gian doc canh bao

        {
            fc::esc::EscCalibrator calib;
            calib.runCalibration(g_esc);
        }

        Serial.println(F("ESC calibration hoan tat."));
        Serial.println(F("Hay tat nguon (ngat pin), doi 5s,"));
        Serial.println(F("sau do dat ESC_CALIBRATION_MODE = 0 va upload lai code de bay binh thuong."));

        // Dung tai day, khong start task nao ca
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

    // Init GPS UART2
    Serial.println(F("[GPS] Init Serial2..."));
    Serial2.begin(cfg::GPS_BAUD, SERIAL_8N1, cfg::GPS_RX_PIN, cfg::GPS_TX_PIN);

    // Setup PID controllers
    setupPidControllers();

    // Motor safety
    g_motorSafety.setMinThrottleForSpin(0.05f);
    g_motorSafety.enableSoftRamp(true);
    g_motorSafety.setSoftRampRate(1.0f);

    // Mixer
    g_mixer.setMinThrottleForSpin(0.02f);

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

    #if PID_TUNE_WEBSOCKET
        // Khởi động WiFi AP + WebSocket PID tuner
        pidtune_ws::init(
            PID_TUNE_WIFI_SSID,              // dùng SSID default "ESP32_PID_TUNER"
            PID_TUNE_WIFI_PASS,              // dùng PASS default "12345678"
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
    // Mọi thứ chạy trong FreeRTOS task; loop() để trống.
    vTaskDelay(portMAX_DELAY);
}
