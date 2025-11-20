
#include "imu_driver.h"

namespace fc {
namespace imu {

Mpu6050Driver::Mpu6050Driver(uint8_t i2cAddress)
    : addr_(i2cAddress)
    , gx_offset_dps_(0.0f)
    , gy_offset_dps_(0.0f)
    , gz_offset_dps_(0.0f)
    , az_offset_ms2_(0.0f)
{}

bool Mpu6050Driver::begin() {
    Wire.begin(fc::cfg::I2C_SDA_PIN, fc::cfg::I2C_SCL_PIN);
    Wire.setClock(fc::cfg::I2C_FREQ_HZ);

    uint8_t who = 0;
    if (!readRegister(REG_WHO_AM_I, who)) {
        return false;
    }
    if (who != WHO_AM_I_EXPECT) {
        return false;
    }

    if (!configure()) return false;
    if (!calibrate(200)) return false;
    return true;
}

bool Mpu6050Driver::configure() {
    if (!writeRegister(REG_PWR_MGMT_1, 0x00)) return false;
    if (!writeRegister(REG_CONFIG, 0x02))     return false; // DLPF ~98Hz
    if (!writeRegister(REG_GYRO_CONFIG, 0x18)) return false; // ±2000 dps
    if (!writeRegister(REG_ACCEL_CONFIG, 0x08)) return false; // ±4g
    if (!writeRegister(REG_SMPLRT_DIV, 0x01))   return false; // 500Hz
    if (!writeRegister(REG_INT_ENABLE, 0x00))   return false;
    return true;
}

bool Mpu6050Driver::calibrate(std::size_t samples) {
    long gx_sum = 0, gy_sum = 0, gz_sum = 0, az_sum = 0;

    for (std::size_t i = 0; i < samples; ++i) {
        uint8_t buf[14];
        if (!readRegisters(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
            return false;
        }
        int16_t az_raw = (static_cast<int16_t>(buf[4]) << 8) | buf[5];
        int16_t gx_raw = (static_cast<int16_t>(buf[8]) << 8) | buf[9];
        int16_t gy_raw = (static_cast<int16_t>(buf[10]) << 8) | buf[11];
        int16_t gz_raw = (static_cast<int16_t>(buf[12]) << 8) | buf[13];

        gx_sum += gx_raw;
        gy_sum += gy_raw;
        gz_sum += gz_raw;
        az_sum += az_raw;

        delay(2);
    }

    float invN = 1.0f / static_cast<float>(samples);
    float gx_mean_raw = static_cast<float>(gx_sum) * invN;
    float gy_mean_raw = static_cast<float>(gy_sum) * invN;
    float gz_mean_raw = static_cast<float>(gz_sum) * invN;
    float az_mean_raw = static_cast<float>(az_sum) * invN;

    gx_offset_dps_ = gx_mean_raw / GYRO_SENS_2000DPS;
    gy_offset_dps_ = gy_mean_raw / GYRO_SENS_2000DPS;
    gz_offset_dps_ = gz_mean_raw / GYRO_SENS_2000DPS;

    float az_mean_g   = az_mean_raw / ACCEL_SENS_4G;
    float az_mean_ms2 = az_mean_g * ONE_G;
    az_offset_ms2_    = az_mean_ms2 - ONE_G;

    return true;
}

bool Mpu6050Driver::read(fc::types::ImuSample& out) {
    uint8_t buf[14];
    if (!readRegisters(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
        return false;
    }

    int16_t ax_raw = (static_cast<int16_t>(buf[0]) << 8) | buf[1];
    int16_t ay_raw = (static_cast<int16_t>(buf[2]) << 8) | buf[3];
    int16_t az_raw = (static_cast<int16_t>(buf[4]) << 8) | buf[5];
    int16_t gx_raw = (static_cast<int16_t>(buf[8]) << 8) | buf[9];
    int16_t gy_raw = (static_cast<int16_t>(buf[10]) << 8) | buf[11];
    int16_t gz_raw = (static_cast<int16_t>(buf[12]) << 8) | buf[13];

    float ax_g = static_cast<float>(ax_raw) / ACCEL_SENS_4G;
    float ay_g = static_cast<float>(ay_raw) / ACCEL_SENS_4G;
    float az_g = static_cast<float>(az_raw) / ACCEL_SENS_4G;

    float ax_ms2 = ax_g * ONE_G;
    float ay_ms2 = ay_g * ONE_G;
    float az_ms2 = az_g * ONE_G;
    az_ms2 -= az_offset_ms2_;

    float gx_dps = static_cast<float>(gx_raw) / GYRO_SENS_2000DPS - gx_offset_dps_;
    float gy_dps = static_cast<float>(gy_raw) / GYRO_SENS_2000DPS - gy_offset_dps_;
    float gz_dps = static_cast<float>(gz_raw) / GYRO_SENS_2000DPS - gz_offset_dps_;

    out.ax = ax_ms2;
    out.ay = ay_ms2;
    out.az = az_ms2;
    out.gx = gx_dps;
    out.gy = gy_dps;
    out.gz = gz_dps;
    out.stamp_us = static_cast<uint32_t>(esp_timer_get_time());
    return true;
}

bool Mpu6050Driver::writeRegister(uint8_t reg,uint8_t value) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(value);
    auto err = Wire.endTransmission(true);
    return (err == 0);
}

bool Mpu6050Driver::readRegisters(uint8_t startReg,uint8_t* buffer,size_t length) {
    Wire.beginTransmission(addr_);
    Wire.write(startReg);
    auto err = Wire.endTransmission(false);
    if (err != 0) return false;

    std::size_t readLen = Wire.requestFrom(addr_, static_cast<uint8_t>(length), static_cast<uint8_t>(true));
    if (readLen != length) return false;

    for (std::size_t i = 0; i < length; ++i) {
        if (!Wire.available()) return false;
        buffer[i] = Wire.read();
    }
    return true;
}

bool Mpu6050Driver::readRegister(uint8_t reg,uint8_t& value) {
    uint8_t buf = 0;
    if (!readRegisters(reg, &buf, 1)) return false;
    value = buf;
    return true;
}

} // namespace imu
} // namespace fc
