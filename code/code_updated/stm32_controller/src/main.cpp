#include <Arduino.h>
#include <Wire.h>
#include "robot_protocol.h"
#include "as5600.hpp"
#include "mpu6050.hpp"
#include "stepper_controller.hpp"
#include "diff_drive.hpp"

// ==========================================
// ĐỊNH NGHĨA CHÂN ĐIỀU KHIỂN (STM32 PIN CONFIG)
// ==========================================
// Động cơ bước Trái (Left Stepper - Driver A4988 / DRV8825)
#define PIN_LEFT_STEP   PA0
#define PIN_LEFT_DIR    PA1
#define PIN_LEFT_EN     PA2

// Động cơ bước Phải (Right Stepper)
#define PIN_RIGHT_STEP  PA3
#define PIN_RIGHT_DIR   PA4
#define PIN_RIGHT_EN    PA5

// Cấu hình giao tiếp I2C
// Bus I2C1: PB6 (SCL), PB7 (SDA) -> MPU6050 (0x68) & AS5600 Bánh Trái (0x36)
// Bus I2C2: PB10 (SCL), PB11 (SDA) -> AS5600 Bánh Phải (0x36)
TwoWire WireLeft(PB7, PB6);     // I2C1
TwoWire WireRight(PB11, PB10);  // I2C2

// Giao tiếp UART với ESP32 (Serial1: PA9 TX, PA10 RX)
#define ESP32_SERIAL Serial1
#define ESP32_BAUDRATE 115200

// ==========================================
// KHỞI TẠO CÁC ĐỐI TƯỢNG HỆ THỐNG
// ==========================================
// Động cơ bước: 3200 xung/vòng (1/16 vi bước)
StepperMotor left_stepper(PIN_LEFT_STEP, PIN_LEFT_DIR, PIN_LEFT_EN, 3200, false);
StepperMotor right_stepper(PIN_RIGHT_STEP, PIN_RIGHT_DIR, PIN_RIGHT_EN, 3200, true);

// Cảm biến góc từ tính AS5600 cho 2 bánh
AS5600Encoder left_encoder(&WireLeft, false);
AS5600Encoder right_encoder(&WireRight, true);

// Cảm biến quán tính MPU6050 (chung trên I2C1)
MPU6050Sensor imu(&WireLeft);

// Bộ điều khiển động học vi sai (khớp thông số 3D RBSlam.step: R = 0.0325m, WheelBase = 0.137m)
DiffDriveController robot_controller(&left_stepper, &right_stepper,
                                     &left_encoder, &right_encoder,
                                     &imu, 0.0325f, 0.137f);

// ==========================================
// CẤU HÌNH NGẮT TIMER PHÁT XUNG BƯỚC (20kHz)
// ==========================================
#if defined(TIM2)
HardwareTimer* StepTimer = nullptr;
void onTimerInterrupt() {
    left_stepper.stepISR();
    right_stepper.stepISR();
}
#endif

// ==========================================
// BIẾN QUẢN LÝ TRUYỀN THÔNG & AN TOÀN
// ==========================================
uint32_t last_control_time_ms = 0;
uint32_t last_cmd_vel_recv_time_ms = 0;
const uint32_t CMD_VEL_TIMEOUT_MS = 500; // Tự động dừng xe nếu mất lệnh sau 500ms

void setup() {
    // Khởi tạo UART debug và UART kết nối ESP32
    Serial.begin(115200);
    ESP32_SERIAL.begin(ESP32_BAUDRATE);

    delay(1000);
    Serial.println("=== KHOI DONG HE THONG XE TU HANH STM32 ===");

    // Khởi tạo các chân I2C
    WireLeft.begin();
    WireRight.begin();

    // Khởi động toàn bộ cảm biến và hệ dẫn động
    robot_controller.begin();

    // Cấu hình ngắt Timer tạo xung bước 20kHz (chu kỳ 50 microseconds)
#if defined(TIM2)
    StepTimer = new HardwareTimer(TIM2);
    StepTimer->pause();
    StepTimer->setOverflow(20000, HERTZ_FORMAT);
    StepTimer->attachInterrupt(onTimerInterrupt);
    StepTimer->resume();
#endif

    last_control_time_ms = millis();
    last_cmd_vel_recv_time_ms = millis();

    Serial.println("He thong da san sang!");
}

void processIncomingCmdVel() {
    // Parser gói tin CmdVelPacket từ ESP32 qua UART
    static uint8_t rx_buffer[sizeof(CmdVelPacket)];
    static size_t rx_idx = 0;

    while (ESP32_SERIAL.available() > 0) {
        uint8_t byte = ESP32_SERIAL.read();

        if (rx_idx == 0) {
            // Tìm byte đầu của header (0x55)
            if (byte == (CMD_VEL_HEADER & 0xFF)) {
                rx_buffer[rx_idx++] = byte;
            }
        } else if (rx_idx == 1) {
            // Kiểm tra byte thứ 2 của header (0xAA)
            if (byte == ((CMD_VEL_HEADER >> 8) & 0xFF)) {
                rx_buffer[rx_idx++] = byte;
            } else if (byte == (CMD_VEL_HEADER & 0xFF)) {
                rx_idx = 1; // Giữ lại byte 0x55 nếu nhận liên tiếp
            } else {
                rx_idx = 0; // Sai header, reset
            }
        } else {
            rx_buffer[rx_idx++] = byte;
            if (rx_idx == sizeof(CmdVelPacket)) {
                // Đã nhận đủ độ dài gói tin, kiểm tra checksum
                CmdVelPacket* packet = reinterpret_cast<CmdVelPacket*>(rx_buffer);
                uint8_t expected_cs = calculate_xor_checksum(rx_buffer, sizeof(CmdVelPacket) - 1);

                if (packet->checksum == expected_cs) {
                    // Cập nhật tốc độ mới cho xe
                    robot_controller.setTargetSpeed(packet->linear_x, packet->angular_z);
                    last_cmd_vel_recv_time_ms = millis();
                }

                rx_idx = 0; // Reset chuẩn bị cho gói kế tiếp
            }
        }
    }
}

void loop() {
    // 1. Nhận lệnh điều khiển từ ESP32
    processIncomingCmdVel();

    // 2. Cơ chế an toàn (Watchdog): dừng xe nếu mất lệnh từ ROS2 quá 500ms
    if (millis() - last_cmd_vel_recv_time_ms > CMD_VEL_TIMEOUT_MS) {
        robot_controller.setTargetSpeed(0.0f, 0.0f);
    }

    // 3. Vòng lặp điều khiển định kỳ 50Hz (mỗi 20ms)
    uint32_t current_time_ms = millis();
    if (current_time_ms - last_control_time_ms >= 20) {
        float dt = (current_time_ms - last_control_time_ms) / 1000.0f;
        last_control_time_ms = current_time_ms;

        // Cập nhật Odometry từ AS5600 + IMU MPU6050
        robot_controller.update(dt);

        // Đóng gói và gửi OdomImuPacket lên ESP32 qua UART
        OdomImuPacket packet;
        robot_controller.populateOdomPacket(packet);

        ESP32_SERIAL.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(OdomImuPacket));
    }
}
