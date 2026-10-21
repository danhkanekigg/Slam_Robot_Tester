#ifndef AS5600_HPP
#define AS5600_HPP

#include <stdint.h>
#include <Wire.h>

/**
 * @brief Lớp điều khiển cảm biến góc từ tính AS5600 (giao tiếp I2C)
 *        Dùng để đọc góc quay và xác định quãng đường/vận tốc của bánh xe.
 */
class AS5600Encoder {
public:
    static constexpr uint8_t AS5600_I2C_ADDR = 0x36;
    static constexpr uint8_t REG_RAW_ANGLE_H = 0x0C;
    static constexpr uint8_t REG_RAW_ANGLE_L = 0x0D;
    static constexpr uint8_t REG_STATUS      = 0x0B;
    static constexpr int32_t COUNTS_PER_REV  = 4096; // 12-bit ADC (0 - 4095)

    /**
     * @param wire_bus Con trỏ tới bộ I2C (Wire hoặc Wire1 trên STM32)
     * @param reversed Đảo chiều góc quay nếu gắn encoder ngược chiều tiến
     */
    AS5600Encoder(TwoWire* wire_bus, bool reversed = false);

    bool begin();

    /**
     * @brief Đọc góc thô trực tiếp từ thanh ghi 12-bit (0 -> 4095)
     */
    uint16_t readRawAngle();

    /**
     * @brief Kiểm tra từ trường nam châm gắn trên trục
     * @return true nếu nam châm nằm trong dải làm việc tốt
     */
    bool isMagnetDetected();

    /**
     * @brief Cập nhật vị trí góc liên tục và xử lý chuyển tiếp vòng (rollover)
     *        Cần được gọi định kỳ trong vòng lặp điều khiển (ví dụ 50Hz - 100Hz).
     */
    void update(float dt);

    /**
     * @brief Lấy tổng số xung tích lũy từ khi khởi động (hỗ trợ cả số âm khi quay lùi)
     */
    int64_t getTotalTicks() const;

    /**
     * @brief Lấy tổng góc quay tích lũy (radian)
     */
    float getTotalAngleRad() const;

    /**
     * @brief Lấy vận tốc góc tức thời của bánh xe (rad/s)
     */
    float getVelocityRadPerSec() const;

    /**
     * @brief Reset lại vị trí đếm xung về 0
     */
    void reset();

private:
    TwoWire* wire_;
    bool reversed_;
    int32_t last_raw_angle_;
    int64_t total_ticks_;
    float velocity_rad_s_;
    bool is_first_read_;

    uint8_t readRegister8(uint8_t reg);
    uint16_t readRegister16(uint8_t reg);
};

#endif // AS5600_HPP

