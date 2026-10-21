#ifndef MPU6050_HPP
#define MPU6050_HPP

#include <stdint.h>
#include <Wire.h>

/**
 * @brief Lớp điều khiển cảm biến quán tính 6 bậc tự do (6-DOF IMU) MPU6050
 *        Cung cấp dữ liệu gia tốc tuyến tính, vận tốc góc và tư thế (Roll, Pitch, Yaw).
 */
class MPU6050Sensor {
public:
    static constexpr uint8_t MPU6050_ADDR = 0x68; // Địa chỉ I2C khi chân AD0 nối GND
    static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
    static constexpr uint8_t REG_CONFIG       = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I     = 0x75;

    struct ImuData {
        float accel_x, accel_y, accel_z; // Đơn vị: m/s^2
        float gyro_x, gyro_y, gyro_z;    // Đơn vị: rad/s
        float roll, pitch, yaw;          // Đơn vị: rad
    };

    /**
     * @param wire_bus Con trỏ tới bộ I2C (Wire hoặc Wire1)
     * @param filter_alpha Trọng số bộ lọc bù (mặc định 0.98 cho Gyro, 0.02 cho Accel)
     */
    MPU6050Sensor(TwoWire* wire_bus, float filter_alpha = 0.98f);

    bool begin();

    /**
     * @brief Tự động hiệu chuẩn (Offset Calibration) khi robot nằm yên trên mặt phẳng
     * @param samples Số lượng mẫu đọc để tính trung bình
     */
    void calibrate(int samples = 500);

    /**
     * @brief Đọc cảm biến và tính toán bộ lọc tư thế
     * @param dt Thời gian trôi qua giữa 2 lần đọc (giây)
     */
    void update(float dt);

    const ImuData& getData() const { return data_; }

private:
    TwoWire* wire_;
    float alpha_;
    ImuData data_;

    // Sai số tĩnh (Offset)
    float accel_bias_x_, accel_bias_y_, accel_bias_z_;
    float gyro_bias_x_, gyro_bias_y_, gyro_bias_z_;

    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};

#endif // MPU6050_HPP

