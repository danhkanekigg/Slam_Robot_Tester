#ifndef DIFF_DRIVE_HPP
#define DIFF_DRIVE_HPP

#include "stepper_controller.hpp"
#include "as5600.hpp"
#include "mpu6050.hpp"
#include "robot_protocol.h"

/**
 * @brief Điều khiển động học xe vi sai 2 bánh (Differential Drive Kinematics)
 *        Tính toán vận tốc 2 bánh xe và tính toán Odometry (vị trí X, Y, Theta).
 *        Được đồng bộ chính xác với kích thước thiết kế 3D (RBSlam.step).
 */
class DiffDriveController {
public:
    /**
     * @param left_motor Con trỏ động cơ bước bánh trái
     * @param right_motor Con trỏ động cơ bước bánh phải
     * @param left_enc Con trỏ cảm biến góc AS5600 bánh trái
     * @param right_enc Con trỏ cảm biến góc AS5600 bánh phải
     * @param imu Con trỏ cảm biến MPU6050
     * @param wheel_radius Bán kính bánh xe (m, 32.5mm => R = 0.0325m)
     * @param wheel_base Khoảng cách giữa 2 tâm bánh xe theo 3D (m, 137mm => L = 0.137m)
     */
    DiffDriveController(StepperMotor* left_motor, StepperMotor* right_motor,
                        AS5600Encoder* left_enc, AS5600Encoder* right_enc,
                        MPU6050Sensor* imu,
                        float wheel_radius = 0.0325f, float wheel_base = 0.137f);

    void begin();

    /**
     * @brief Đặt vận tốc mong muốn (linear_x m/s, angular_z rad/s)
     */
    void setTargetSpeed(float linear_x, float angular_z);

    /**
     * @brief Cập nhật động học và vị trí Odometry
     * @param dt Thời gian chu kỳ (giây)
     */
    void update(float dt);

    /**
     * @brief Đóng gói dữ liệu Odometry và IMU vào struct gửi đi
     */
    void populateOdomPacket(OdomImuPacket& packet);

    // Getters
    float getX() const { return x_; }
    float getY() const { return y_; }
    float getTheta() const { return theta_; }
    float getLinearVelocity() const { return v_actual_; }
    float getAngularVelocity() const { return w_actual_; }

    void resetOdometry();

    /**
     * @brief Điều chỉnh kích thước thực tế của xe 2 bánh chủ động
     * @param wheel_radius Bán kính bánh chủ động (m)
     * @param wheel_base Khoảng cách giữa 2 bánh chủ động (m)
     */
    void setWheelDimensions(float wheel_radius, float wheel_base);

    /**
     * @brief Điều chỉnh trọng số kết hợp IMU Gyro Z và Odometry bánh xe
     *        Đối với xe 2 bánh chủ động + 1 bánh tự do, khi quay gấp bánh tự do
     *        có thể tạo kháng lực xoay làm trượt nhẹ bánh xe, nên ưu tiên Gyro Z (0.7 - 0.9).
     * @param imu_weight Trọng số từ 0.0 (hoàn toàn theo bánh) đến 1.0 (hoàn toàn theo Gyro Z)
     */
    void setImuFusionWeight(float imu_weight);

private:
    StepperMotor* left_motor_;
    StepperMotor* right_motor_;
    AS5600Encoder* left_enc_;
    AS5600Encoder* right_enc_;
    MPU6050Sensor* imu_;

    float wheel_radius_;
    float wheel_base_;

    // Tọa độ xe
    float x_;
    float y_;
    float theta_;

    float v_actual_;
    float w_actual_;

    float last_left_angle_rad_;
    float last_right_angle_rad_;
    bool is_first_update_;
    float imu_fusion_weight_;
};

#endif // DIFF_DRIVE_HPP
