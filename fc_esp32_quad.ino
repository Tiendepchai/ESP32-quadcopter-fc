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
#define PID_TUNE_ALLOW_WHEN_ARMED 1
#define PID_TUNE_ARMED_REQUIRE_MODE 0
#define PID_TUNE_ARMED_MIN_THROTTLE 0.20f
#define PID_TUNE_ARMED_MAX_THROTTLE 0.60f
#define PID_TUNE_ARMED_MAX_STICK 0.10f
#define PID_TUNE_ARMED_APPLY_COOLDOWN_MS 1800u
#define PID_TUNE_MAX_STEP_KP 0.025f
#define PID_TUNE_MAX_STEP_KI 0.012f
#define PID_TUNE_MAX_STEP_KD 0.004f

#define PID_TUNE_WIFI_SSID "ESP32_PID_TUNER"
#define PID_TUNE_WIFI_PASS "12345678"

#include "pid_tune_udp.h"   // module UDP PID tuner

#include "idle_finder.h"

// 0 = bay bình thường, 1 = chạy tool tìm idle
#define IDLE_FINDER_MODE 0

#define ESC_CALIBRATION_MODE 0   // Đổi thành 1 khi muốn calibrate ESC
#define SERIAL_STATUS_LOG 1
#define SERIAL_STATUS_LOG_INTERVAL_MS 500u

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
volatile bool g_statusLogEnabled = false;

const char* controlModeName(const fc::types::ControlInput& rc) {
    if (!rc.arm) {
        return modes::FlightModeManager::modeName(modes::Mode::DISARM);
    }
    switch (rc.mode) {
    case 0: return modes::FlightModeManager::modeName(modes::Mode::ANGLE);
    case 1: return modes::FlightModeManager::modeName(modes::Mode::ACRO);
    case 2: return "MODE2";
    default: return "MODE?";
    }
}

void printStatusSnapshot() {
    fc::types::ControlInput rc{};
    fc::types::FusedAttitude att{};
    fc::types::RateTelemetry rate{};
    fc::types::BatteryState bat{};
    fc::types::GpsData gps{};

    const bool hasRc = g_bus.getControlInput(rc);
    const bool hasAtt = g_bus.getAttitude(att);
    const bool hasRate = g_bus.getRateTelemetry(rate);
    const bool hasBat = g_bus.getBatteryState(bat);
    const bool hasGps = g_bus.getGpsData(gps);

    float rpKp, rpKi, rpKd, yKp, yKi, yKd;
    fc::pidtune_udp::getCurrentPid(rpKp, rpKi, rpKd, yKp, yKi, yKd);

    const char* tuneReason = "NO_RC";
    bool tuneOk = false;
    if (hasRc) {
        tuneOk = fc::pidtune_udp::getTuneWindowStatus(rc, tuneReason);
        if (tuneOk) {
            tuneReason = "OK";
        } else if (!rc.arm) {
            tuneReason = "DISARMED";
        } else if (!tuneReason || !tuneReason[0]) {
            tuneReason = "BLOCKED";
        }
    }

    char line[512];
    snprintf(
        line,
        sizeof(line),
        "[STAT] ms=%lu rc=%d att=%d rate=%d bat=%d gps=%d mode=%s mode_sw=%u arm=%d "
        "thr=%.3f sticks=(%.2f,%.2f,%.2f) att_deg=(%.1f,%.1f,%.1f) "
        "rate_dps=(%.1f/%.1f,%.1f/%.1f,%.1f/%.1f) "
        "pid_rp=(%.4f,%.4f,%.4f) pid_y=(%.4f,%.4f,%.4f) tune=%s "
        "vbat=%.2f gps_fix=%d sats=%u",
        static_cast<unsigned long>(millis()),
        hasRc ? 1 : 0,
        hasAtt ? 1 : 0,
        hasRate ? 1 : 0,
        hasBat ? 1 : 0,
        hasGps ? 1 : 0,
        hasRc ? controlModeName(rc) : "NO_RC",
        hasRc ? static_cast<unsigned>(rc.mode) : 0u,
        hasRc ? (rc.arm ? 1 : 0) : 0,
        hasRc ? rc.throttle : 0.0f,
        hasRc ? rc.roll : 0.0f,
        hasRc ? rc.pitch : 0.0f,
        hasRc ? rc.yaw : 0.0f,
        hasAtt ? att.roll_deg : 0.0f,
        hasAtt ? att.pitch_deg : 0.0f,
        hasAtt ? att.yaw_deg : 0.0f,
        hasRate ? rate.roll_rate_dps : 0.0f,
        hasRate ? rate.roll_setpoint_dps : 0.0f,
        hasRate ? rate.pitch_rate_dps : 0.0f,
        hasRate ? rate.pitch_setpoint_dps : 0.0f,
        hasRate ? rate.yaw_rate_dps : 0.0f,
        hasRate ? rate.yaw_setpoint_dps : 0.0f,
        rpKp, rpKi, rpKd,
        yKp, yKi, yKd,
        tuneReason,
        hasBat ? bat.vbat : 0.0f,
        (hasGps && gps.fix) ? 1 : 0,
        hasGps ? static_cast<unsigned>(gps.sats) : 0u
    );
    Serial.println(line);
}

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

#if SERIAL_STATUS_LOG
        uint32_t now = millis();
        if (g_statusLogEnabled && now - lastPrintMs >= SERIAL_STATUS_LOG_INTERVAL_MS) {
            lastPrintMs = now;
            printStatusSnapshot();
        }
#endif
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
    Serial.println(F("[RC] Initializing PWM receiver..."));
    if (!g_rcRx.begin()) {
        Serial.println(F("[RC] ERROR: PWM receiver init failed."));
    } else {
        Serial.println(F("[RC] OK."));
    }
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

    Serial.println(F("[LOG] [STAT] ms rc att rate bat gps mode mode_sw arm thr sticks att_deg rate_dps(meas/sp) pid_rp pid_y tune vbat gps_fix sats"));
    Serial.println(F("===== FC ESP32 QUAD - READY ====="));

    // Start FreeRTOS tasks
    g_statusLogEnabled = true;
    g_fcRunner.start();
    g_ioRunner.start();
}

void loop() {
    #if PID_TUNE_UDP
        // xử lý UDP PID tuner (gửi ATT + nhận PID) trong loopTask
        fc::pidtune_udp::update();
    #endif

    // nhường CPU cho các task khác (WiFi, flight-control, v.v.)
    vTaskDelay(1);   // hoặc delay(1);
}
