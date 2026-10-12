#include "mpu6050.hpp"
#include <Arduino.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Hệ số chuyển đổi
// Gyro dải đo +-250 deg/s => 131.0 LSB/(deg/s)
static constexpr float GYRO_SCALE_DEG_S = 1.0f / 131.0f;
static constexpr float DEG_TO_RAD_F     = M_PI / 180.0f;
// Accel dải đo +-2g => 16384 LSB/g. 1g = 9.80665 m/s^2
static constexpr float ACCEL_SCALE_MS2  = 9.80665f / 16384.0f;

MPU6050Sensor::MPU6050Sensor(TwoWire* wire_bus, float filter_alpha)
    : wire_(wire_bus),
      alpha_(filter_alpha),
      data_{0},
      accel_bias_x_(0), accel_bias_y_(0), accel_bias_z_(0),
      gyro_bias_x_(0), gyro_bias_y_(0), gyro_bias_z_(0) {}

void MPU6050Sensor::writeRegister(uint8_t reg, uint8_t value) {
    wire_->beginTransmission(MPU6050_ADDR);
    wire_->write(reg);
    wire_->write(value);
    wire_->endTransmission();
}

uint8_t MPU6050Sensor::readRegister(uint8_t reg) {
    wire_->beginTransmission(MPU6050_ADDR);
    wire_->write(reg);
    wire_->endTransmission(false);
    wire_->requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    if (wire_->available()) {
        return wire_->read();
    }
    return 0;
}

bool MPU6050Sensor::begin() {
    if (!wire_) return false;
    wire_->begin();

    // Đánh thức MPU6050 (thoát khỏi chế độ Sleep)
    writeRegister(REG_PWR_MGMT_1, 0x00);
    delay(10);

    // Kiểm tra WHO_AM_I (mặc định phải trả về 0x68)
    uint8_t who_am_i = readRegister(REG_WHO_AM_I);
    if (who_am_i != 0x68) {
        return false;
    }

    // Cấu hình Sample Rate Divider = 0 (1kHz)
    writeRegister(REG_SMPLRT_DIV, 0x00);

    // Cấu hình bộ lọc thông thấp số (DLPF = 44Hz)
    writeRegister(REG_CONFIG, 0x03);

    // Cấu hình dải đo Gyro: +- 250 deg/s (FS_SEL = 0)
    writeRegister(REG_GYRO_CONFIG, 0x00);

    // Cấu hình dải đo Accel: +- 2g (AFS_SEL = 0)
    writeRegister(REG_ACCEL_CONFIG, 0x00);

    return true;
}

void MPU6050Sensor::calibrate(int samples) {
    if (samples <= 0) return;

    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    int valid_samples = 0;
    for (int i = 0; i < samples; ++i) {
        wire_->beginTransmission(MPU6050_ADDR);
        wire_->write(REG_ACCEL_XOUT_H);
        wire_->endTransmission(false);
        wire_->requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);

        if (wire_->available() >= 14) {
            int16_t raw_ax = (wire_->read() << 8) | wire_->read();
            int16_t raw_ay = (wire_->read() << 8) | wire_->read();
            int16_t raw_az = (wire_->read() << 8) | wire_->read();
            wire_->read(); wire_->read(); // Bỏ qua nhiệt độ
            int16_t raw_gx = (wire_->read() << 8) | wire_->read();
            int16_t raw_gy = (wire_->read() << 8) | wire_->read();
            int16_t raw_gz = (wire_->read() << 8) | wire_->read();

            sum_ax += (float)raw_ax * ACCEL_SCALE_MS2;
            sum_ay += (float)raw_ay * ACCEL_SCALE_MS2;
            sum_az += (float)raw_az * ACCEL_SCALE_MS2;

            sum_gx += (float)raw_gx * GYRO_SCALE_DEG_S * DEG_TO_RAD_F;
            sum_gy += (float)raw_gy * GYRO_SCALE_DEG_S * DEG_TO_RAD_F;
            sum_gz += (float)raw_gz * GYRO_SCALE_DEG_S * DEG_TO_RAD_F;
            valid_samples++;
        }
        delay(2);
    }

    if (valid_samples > 0) {
        accel_bias_x_ = sum_ax / valid_samples;
        accel_bias_y_ = sum_ay / valid_samples;
        // Trọng lực theo trục Z khi nằm ngang là +9.80665 m/s^2
        accel_bias_z_ = (sum_az / valid_samples) - 9.80665f;

        gyro_bias_x_ = sum_gx / valid_samples;
        gyro_bias_y_ = sum_gy / valid_samples;
        gyro_bias_z_ = sum_gz / valid_samples;
    }
}

void MPU6050Sensor::update(float dt) {
    if (dt <= 0.0f) return;

    wire_->beginTransmission(MPU6050_ADDR);
    wire_->write(REG_ACCEL_XOUT_H);
    wire_->endTransmission(false);
    wire_->requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);

    if (wire_->available() >= 14) {
        int16_t raw_ax = (wire_->read() << 8) | wire_->read();
        int16_t raw_ay = (wire_->read() << 8) | wire_->read();
        int16_t raw_az = (wire_->read() << 8) | wire_->read();
        wire_->read(); wire_->read(); // Temp
        int16_t raw_gx = (wire_->read() << 8) | wire_->read();
        int16_t raw_gy = (wire_->read() << 8) | wire_->read();
        int16_t raw_gz = (wire_->read() << 8) | wire_->read();

        // Trừ độ lệch tĩnh
        data_.accel_x = ((float)raw_ax * ACCEL_SCALE_MS2) - accel_bias_x_;
        data_.accel_y = ((float)raw_ay * ACCEL_SCALE_MS2) - accel_bias_y_;
        data_.accel_z = ((float)raw_az * ACCEL_SCALE_MS2) - accel_bias_z_;

        data_.gyro_x = (((float)raw_gx * GYRO_SCALE_DEG_S) * DEG_TO_RAD_F) - gyro_bias_x_;
        data_.gyro_y = (((float)raw_gy * GYRO_SCALE_DEG_S) * DEG_TO_RAD_F) - gyro_bias_y_;
        data_.gyro_z = (((float)raw_gz * GYRO_SCALE_DEG_S) * DEG_TO_RAD_F) - gyro_bias_z_;

        // Tính góc nghiêng tĩnh từ gia tốc kế
        float roll_acc = atan2f(data_.accel_y, data_.accel_z);
        float pitch_acc = atan2f(-data_.accel_x, sqrtf(data_.accel_y * data_.accel_y + data_.accel_z * data_.accel_z));

        // Bộ lọc bù (Complementary Filter)
        data_.roll  = alpha_ * (data_.roll + data_.gyro_x * dt) + (1.0f - alpha_) * roll_acc;
        data_.pitch = alpha_ * (data_.pitch + data_.gyro_y * dt) + (1.0f - alpha_) * pitch_acc;
        
        // Yaw tích phân từ vận tốc góc Z (loại trừ trôi bằng cách chuẩn hóa trong dải -PI đến PI)
        data_.yaw += data_.gyro_z * dt;
        while (data_.yaw > (float)M_PI)  data_.yaw -= 2.0f * (float)M_PI;
        while (data_.yaw < -(float)M_PI) data_.yaw += 2.0f * (float)M_PI;
    }
}

