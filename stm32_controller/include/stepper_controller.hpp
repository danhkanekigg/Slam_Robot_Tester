#ifndef STEPPER_CONTROLLER_HPP
#define STEPPER_CONTROLLER_HPP

#include <stdint.h>
#include <Arduino.h>

/**
 * @brief Điều khiển 1 động cơ bước NEMA 17 sử dụng Driver A4988/DRV8825/TMC2209 (Giao tiếp STEP/DIR)
 *        Bao gồm cơ chế tạo xung, giới hạn gia tốc để chống mất bước và rung động.
 */
class StepperMotor {
public:
    /**
     * @param step_pin Chân phát xung STEP
     * @param dir_pin Chân chọn chiều DIR
     * @param en_pin Chân kích hoạt ENABLE (Active LOW)
     * @param steps_per_rev Số bước trên 1 vòng (ví dụ 1.8 deg = 200 bước x 16 vi bước = 3200)
     * @param reversed Đảo chiều quay nếu động cơ lắp ngược
     */
    StepperMotor(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin,
                 uint32_t steps_per_rev = 3200, bool reversed = false);

    void begin();
    void enable(bool state = true);

    /**
     * @brief Đặt vận tốc mục tiêu (rad/s)
     */
    void setTargetVelocity(float rad_per_sec);

    /**
     * @brief Đặt gia tốc tối đa (rad/s^2)
     */
    void setMaxAcceleration(float max_accel_rad_s2);

    /**
     * @brief Cập nhật vận tốc mượt mà theo gia tốc (gọi trong vòng lặp điều khiển, ví dụ 100Hz)
     * @param dt Thời gian chu kỳ (giây)
     */
    void updateSpeed(float dt);

    /**
     * @brief Hàm phát xung bước cực nhanh (được gọi từ ISR ngắt Timer)
     *        Thực hiện tạo xung STEP khi đến chu kỳ xung tương ứng với vận tốc hiện tại.
     */
    inline void stepISR() {
        if (current_step_interval_us_ == 0) return; // Động cơ đang dừng

        step_counter_us_ += timer_isr_period_us_;
        if (step_counter_us_ >= current_step_interval_us_) {
            step_counter_us_ -= current_step_interval_us_;
            // Tạo xung dương (HIGH -> LOW) với độ rộng xung tối thiểu 2us
            digitalWrite(step_pin_, HIGH);
            delayMicroseconds(2);
            digitalWrite(step_pin_, LOW);
            
            if (current_dir_forward_) {
                step_position_++;
            } else {
                step_position_--;
            }
        }
    }

    /**
     * @brief Đặt chu kỳ gọi ISR (mặc định ngắt Timer chạy ở tần số 20kHz => 50us)
     */
    void setISRPeriodUs(uint32_t isr_period_us);

    float getCurrentVelocity() const { return current_velocity_; }
    int64_t getStepPosition() const { return step_position_; }

private:
    uint8_t step_pin_;
    uint8_t dir_pin_;
    uint8_t en_pin_;
    uint32_t steps_per_rev_;
    bool reversed_;

    float target_velocity_;   // rad/s
    float current_velocity_;  // rad/s
    float max_accel_;         // rad/s^2

    volatile uint32_t current_step_interval_us_; // Khoảng thời gian giữa 2 xung (micro-seconds)
    volatile uint32_t step_counter_us_;
    volatile uint32_t timer_isr_period_us_;
    volatile bool current_dir_forward_;
    volatile int64_t step_position_;
};

#endif // STEPPER_CONTROLLER_HPP

