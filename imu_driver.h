
#pragma once

#include <cstdint>
#include <cstddef>

#include <Arduino.h>
#include <Wire.h>
#include "esp_timer.h"

#include "platform_config.h"
#include "shared_types.h"

namespace fc {
namespace imu {

struct IImuDriver {
    virtual bool begin() = 0;
    virtual bool read(fc::types::ImuSample& out) = 0;
    virtual ~IImuDriver() = default;
};

class Mpu6050Driver : public IImuDriver {
public:
    explicit Mpu6050Driver(uint8_t i2cAddress = 0x68);

    bool begin() override;
    bool read(fc::types::ImuSample& out) override;
    bool runCalibration(std::size_t samples = 1000);
private:
    uint8_t addr_;
    float gx_offset_dps_;
    float gy_offset_dps_;
    float gz_offset_dps_;
    float az_offset_ms2_;

    static constexpr float GYRO_SENS_2000DPS = 16.4f;
    static constexpr float ACCEL_SENS_4G     = 8192.0f;
    static constexpr float ONE_G             = 9.80665f;

    static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
    static constexpr uint8_t REG_CONFIG       = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_INT_ENABLE   = 0x38;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I     = 0x75;

    static constexpr uint8_t WHO_AM_I_EXPECT  = 0x68;

    bool writeRegister(uint8_t reg,uint8_t value);
    bool readRegisters(uint8_t startReg,uint8_t* buffer,size_t length);
    bool readRegister(uint8_t reg,uint8_t& value);

    bool configure();
    bool calibrate(std::size_t samples = 500);
};

} // namespace imu
} // namespace fc
