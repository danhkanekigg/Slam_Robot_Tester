#include "stepper_controller.hpp"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

StepperMotor::StepperMotor(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin,
                           uint32_t steps_per_rev, bool reversed)
    : step_pin_(step_pin),
      dir_pin_(dir_pin),
      en_pin_(en_pin),
      steps_per_rev_(steps_per_rev),
      reversed_(reversed),
      target_velocity_(0.0f),
      current_velocity_(0.0f),
      max_accel_(10.0f), // 10 rad/s^2 mặc định
      current_step_interval_us_(0),
      step_counter_us_(0),
      timer_isr_period_us_(50), // Chu kỳ ngắt 50us (20kHz)
      current_dir_forward_(true),
      step_position_(0) {}

void StepperMotor::begin() {
    pinMode(step_pin_, OUTPUT);
    pinMode(dir_pin_, OUTPUT);
    pinMode(en_pin_, OUTPUT);

    digitalWrite(step_pin_, LOW);
    digitalWrite(dir_pin_, LOW);
    enable(true);
}

void StepperMotor::enable(bool state) {
    // Driver A4988 / DRV8825: Chân EN tích cực mức THẤP (LOW = BẬT, HIGH = TẮT)
    digitalWrite(en_pin_, state ? LOW : HIGH);
}

void StepperMotor::setTargetVelocity(float rad_per_sec) {
    target_velocity_ = rad_per_sec;
}

void StepperMotor::setMaxAcceleration(float max_accel_rad_s2) {
    if (max_accel_rad_s2 > 0.1f) {
        max_accel_ = max_accel_rad_s2;
    }
}

void StepperMotor::setISRPeriodUs(uint32_t isr_period_us) {
    timer_isr_period_us_ = isr_period_us;
}

void StepperMotor::updateSpeed(float dt) {
    if (dt <= 0.0f) return;

    // Giới hạn tốc độ biến thiên theo gia tốc tối đa (Ramp)
    float delta_v = target_velocity_ - current_velocity_;
    float max_step = max_accel_ * dt;

    if (fabsf(delta_v) <= max_step) {
        current_velocity_ = target_velocity_;
    } else if (delta_v > 0) {
        current_velocity_ += max_step;
    } else {
        current_velocity_ -= max_step;
    }

    // Xác định chiều quay và thiết lập chân DIR
    bool forward = (current_velocity_ >= 0.0f);
    if (reversed_) {
        forward = !forward;
    }
    current_dir_forward_ = forward;
    digitalWrite(dir_pin_, forward ? HIGH : LOW);

    // Tính tần số xung và khoảng thời gian giữa 2 xung (micro-seconds)
    float speed_abs = fabsf(current_velocity_);
    if (speed_abs < 0.01f) {
        // Tốc độ quá nhỏ coi như dừng
        current_step_interval_us_ = 0;
    } else {
        // Tần số xung = (speed / 2*PI) * steps_per_rev
        float step_freq = (speed_abs / (2.0f * (float)M_PI)) * (float)steps_per_rev_;
        if (step_freq > 0.0f) {
            uint32_t interval = (uint32_t)(1000000.0f / step_freq);
            // Giới hạn tần số tối đa (tối thiểu 20us ~ 50kHz)
            if (interval < 20) interval = 20;
            current_step_interval_us_ = interval;
        } else {
            current_step_interval_us_ = 0;
        }
    }
}

