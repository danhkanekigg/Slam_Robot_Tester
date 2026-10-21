#include "diff_drive.hpp"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

DiffDriveController::DiffDriveController(StepperMotor* left_motor, StepperMotor* right_motor,
                                         AS5600Encoder* left_enc, AS5600Encoder* right_enc,
                                         MPU6050Sensor* imu,
                                         float wheel_radius, float wheel_base)
    : left_motor_(left_motor),
      right_motor_(right_motor),
      left_enc_(left_enc),
      right_enc_(right_enc),
      imu_(imu),
      wheel_radius_(wheel_radius),
      wheel_base_(wheel_base),
      x_(0.0f),
      y_(0.0f),
      theta_(0.0f),
      v_actual_(0.0f),
      w_actual_(0.0f),
      last_left_angle_rad_(0.0f),
      last_right_angle_rad_(0.0f),
      is_first_update_(true),
      imu_fusion_weight_(0.8f) {}

void DiffDriveController::begin() {
    if (left_motor_) left_motor_->begin();
    if (right_motor_) right_motor_->begin();
    if (left_enc_) left_enc_->begin();
    if (right_enc_) right_enc_->begin();
    if (imu_) {
        imu_->begin();
        imu_->calibrate();
    }
    resetOdometry();
}

void DiffDriveController::setTargetSpeed(float linear_x, float angular_z) {
    // Động học nghịch (Inverse Kinematics):
    // v_left  = v - (w * L / 2)
    // v_right = v + (w * L / 2)
    float v_l = linear_x - (angular_z * wheel_base_ / 2.0f);
    float v_r = linear_x + (angular_z * wheel_base_ / 2.0f);

    // Chuyển sang vận tốc góc của từng bánh: omega = v / R (rad/s)
    float w_l = v_l / wheel_radius_;
    float w_r = v_r / wheel_radius_;

    if (left_motor_) left_motor_->setTargetVelocity(w_l);
    if (right_motor_) right_motor_->setTargetVelocity(w_r);
}

void DiffDriveController::update(float dt) {
    if (dt <= 0.0f) return;

    // Cập nhật tốc độ động cơ bước
    if (left_motor_) left_motor_->updateSpeed(dt);
    if (right_motor_) right_motor_->updateSpeed(dt);

    // Cập nhật cảm biến góc AS5600
    if (left_enc_) left_enc_->update(dt);
    if (right_enc_) right_enc_->update(dt);

    // Cập nhật IMU
    if (imu_) imu_->update(dt);

    // Tính Odometry từ góc quay AS5600 (hoặc vị trí bước)
    float cur_left_angle = left_enc_ ? left_enc_->getTotalAngleRad() : 0.0f;
    float cur_right_angle = right_enc_ ? right_enc_->getTotalAngleRad() : 0.0f;

    if (is_first_update_) {
        last_left_angle_rad_ = cur_left_angle;
        last_right_angle_rad_ = cur_right_angle;
        is_first_update_ = false;
        return;
    }

    float d_theta_l = cur_left_angle - last_left_angle_rad_;
    float d_theta_r = cur_right_angle - last_right_angle_rad_;

    last_left_angle_rad_ = cur_left_angle;
    last_right_angle_rad_ = cur_right_angle;

    // Quãng đường lăn của từng bánh xe
    float d_s_l = d_theta_l * wheel_radius_;
    float d_s_r = d_theta_r * wheel_radius_;

    // Quãng đường tịnh tiến trung bình và độ lệch hướng
    float d_s = (d_s_r + d_s_l) / 2.0f;
    float d_theta_wheel = (d_s_r - d_s_l) / wheel_base_;

    // Kết hợp độ xoay từ IMU để tăng độ chính xác (giảm sai số trượt bánh do bánh tự do swivel drag)
    float d_theta = d_theta_wheel;
    if (imu_) {
        // Gyro Z cho vận tốc góc thực tế không bị ảnh hưởng bởi trượt bánh
        float imu_w_z = imu_->getData().gyro_z;
        d_theta = imu_fusion_weight_ * (imu_w_z * dt) + (1.0f - imu_fusion_weight_) * d_theta_wheel;
    }

    // Tích phân Euler Runge-Kutta bậc 2 tính tọa độ vị trí (x, y)
    float half_d_theta = d_theta / 2.0f;
    x_ += d_s * cosf(theta_ + half_d_theta);
    y_ += d_s * sinf(theta_ + half_d_theta);
    theta_ += d_theta;

    // Chuẩn hóa theta trong dải [-PI, PI]
    while (theta_ > (float)M_PI)  theta_ -= 2.0f * (float)M_PI;
    while (theta_ < -(float)M_PI) theta_ += 2.0f * (float)M_PI;

    // Tính vận tốc thực tế
    v_actual_ = d_s / dt;
    w_actual_ = d_theta / dt;
}

void DiffDriveController::populateOdomPacket(OdomImuPacket& packet) {
    packet.header = ODOM_IMU_HEADER;
    packet.timestamp_ms = millis();
    packet.x = x_;
    packet.y = y_;
    packet.theta = theta_;
    packet.linear_v = v_actual_;
    packet.angular_w = w_actual_;

    if (imu_) {
        const auto& imu_data = imu_->getData();
        packet.accel_x = imu_data.accel_x;
        packet.accel_y = imu_data.accel_y;
        packet.accel_z = imu_data.accel_z;
        packet.gyro_x  = imu_data.gyro_x;
        packet.gyro_y  = imu_data.gyro_y;
        packet.gyro_z  = imu_data.gyro_z;
        packet.roll    = imu_data.roll;
        packet.pitch   = imu_data.pitch;
        packet.yaw     = imu_data.yaw;
    } else {
        packet.accel_x = packet.accel_y = packet.accel_z = 0.0f;
        packet.gyro_x  = packet.gyro_y  = packet.gyro_z  = 0.0f;
        packet.roll    = packet.pitch   = 0.0f;
        packet.yaw     = theta_;
    }

    // Tính checksum
    uint8_t* p = reinterpret_cast<uint8_t*>(&packet);
    size_t payload_len = sizeof(OdomImuPacket) - sizeof(packet.checksum);
    packet.checksum = calculate_xor_checksum(p, payload_len);
}

void DiffDriveController::resetOdometry() {
    x_ = 0.0f;
    y_ = 0.0f;
    theta_ = 0.0f;
    v_actual_ = 0.0f;
    w_actual_ = 0.0f;
    if (left_enc_) left_enc_->reset();
    if (right_enc_) right_enc_->reset();
    is_first_update_ = true;
}

void DiffDriveController::setWheelDimensions(float wheel_radius, float wheel_base) {
    if (wheel_radius > 0.005f) wheel_radius_ = wheel_radius;
    if (wheel_base > 0.02f) wheel_base_ = wheel_base;
}

void DiffDriveController::setImuFusionWeight(float imu_weight) {
    if (imu_weight >= 0.0f && imu_weight <= 1.0f) {
        imu_fusion_weight_ = imu_weight;
    }
}
