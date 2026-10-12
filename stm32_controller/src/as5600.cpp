#include "as5600.hpp"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

AS5600Encoder::AS5600Encoder(TwoWire* wire_bus, bool reversed)
    : wire_(wire_bus),
      reversed_(reversed),
      last_raw_angle_(0),
      total_ticks_(0),
      velocity_rad_s_(0.0f),
      is_first_read_(true) {}

bool AS5600Encoder::begin() {
    if (!wire_) return false;
    wire_->begin();
    
    // Kiểm tra kết nối tới địa chỉ I2C của AS5600
    wire_->beginTransmission(AS5600_I2C_ADDR);
    if (wire_->endTransmission() != 0) {
        return false;
    }
    
    last_raw_angle_ = readRawAngle();
    is_first_read_ = true;
    total_ticks_ = 0;
    velocity_rad_s_ = 0.0f;
    return true;
}

uint8_t AS5600Encoder::readRegister8(uint8_t reg) {
    wire_->beginTransmission(AS5600_I2C_ADDR);
    wire_->write(reg);
    wire_->endTransmission(false);
    
    wire_->requestFrom((uint8_t)AS5600_I2C_ADDR, (uint8_t)1);
    if (wire_->available()) {
        return wire_->read();
    }
    return 0;
}

uint16_t AS5600Encoder::readRegister16(uint8_t reg) {
    wire_->beginTransmission(AS5600_I2C_ADDR);
    wire_->write(reg);
    wire_->endTransmission(false);
    
    wire_->requestFrom((uint8_t)AS5600_I2C_ADDR, (uint8_t)2);
    if (wire_->available() >= 2) {
        uint8_t high = wire_->read();
        uint8_t low = wire_->read();
        return ((uint16_t)(high & 0x0F) << 8) | low;
    }
    return 0;
}

uint16_t AS5600Encoder::readRawAngle() {
    return readRegister16(REG_RAW_ANGLE_H);
}

bool AS5600Encoder::isMagnetDetected() {
    uint8_t status = readRegister8(REG_STATUS);
    // Bit 5 = Magnet Detected (MD)
    return (status & 0x20) != 0;
}

void AS5600Encoder::update(float dt) {
    if (dt <= 0.0f) return;

    int32_t current_raw = readRawAngle();

    if (is_first_read_) {
        last_raw_angle_ = current_raw;
        is_first_read_ = false;
        return;
    }

    // Tính độ dịch chuyển góc tức thời delta_raw
    int32_t delta = current_raw - last_raw_angle_;

    // Xử lý hiện tượng tràn vòng tròn (Rollover từ 0 -> 4095 hoặc ngược lại)
    // Nếu nhảy hơn nửa vòng tròn (> 2048 hoặc < -2048) thì xảy ra rollover
    if (delta > (COUNTS_PER_REV / 2)) {
        delta -= COUNTS_PER_REV;
    } else if (delta < -(COUNTS_PER_REV / 2)) {
        delta += COUNTS_PER_REV;
    }

    if (reversed_) {
        delta = -delta;
    }

    total_ticks_ += delta;
    last_raw_angle_ = current_raw;

    // Vận tốc góc = (delta_ticks / 4096) * 2 * PI / dt (rad/s)
    float delta_rad = ((float)delta / (float)COUNTS_PER_REV) * 2.0f * (float)M_PI;
    velocity_rad_s_ = delta_rad / dt;
}

int64_t AS5600Encoder::getTotalTicks() const {
    return total_ticks_;
}

float AS5600Encoder::getTotalAngleRad() const {
    return ((float)total_ticks_ / (float)COUNTS_PER_REV) * 2.0f * (float)M_PI;
}

float AS5600Encoder::getVelocityRadPerSec() const {
    return velocity_rad_s_;
}

void AS5600Encoder::reset() {
    total_ticks_ = 0;
    velocity_rad_s_ = 0.0f;
    is_first_read_ = true;
}

