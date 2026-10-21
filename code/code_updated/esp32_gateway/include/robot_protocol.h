#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#pragma pack(push, 1)

// ==========================================
// CẤU TRÚC GÓI TIN ĐIỀU KHIỂN VẬN TỐC (PC -> ESP32 -> STM32)
// ==========================================
#define CMD_VEL_HEADER 0xAA55

struct CmdVelPacket {
    uint16_t header;      // 0xAA55
    float linear_x;       // Vận tốc dài yêu cầu (m/s)
    float angular_z;      // Vận tốc góc yêu cầu (rad/s)
    uint8_t checksum;     // XOR checksum của payload
};

// ==========================================
// CẤU TRÚC GÓI TIN ODOMETRY & IMU (STM32 -> ESP32 -> PC)
// ==========================================
#define ODOM_IMU_HEADER 0x55AA

struct OdomImuPacket {
    uint16_t header;      // 0x55AA
    uint32_t timestamp_ms;// Thời gian tính bằng mili-giây
    
    // Odometry tính từ bánh xe (AS5600)
    float x;              // Vị trí X trong khung odom (mét)
    float y;              // Vị trí Y trong khung odom (mét)
    float theta;          // Góc hướng xe (radian, -PI đến PI)
    float linear_v;       // Vận tốc dài hiện tại (m/s)
    float angular_w;      // Vận tốc góc hiện tại (rad/s)
    
    // Dữ liệu đo từ MPU6050
    float accel_x;        // Gia tốc tuyến tính X (m/s^2)
    float accel_y;        // Gia tốc tuyến tính Y (m/s^2)
    float accel_z;        // Gia tốc tuyến tính Z (m/s^2)
    float gyro_x;         // Vận tốc góc Roll (rad/s)
    float gyro_y;         // Vận tốc góc Pitch (rad/s)
    float gyro_z;         // Vận tốc góc Yaw (rad/s)
    
    // Góc tư thế được lọc (Complementary filter)
    float roll;           // Góc nghiêng lật (rad)
    float pitch;          // Góc chúc ngẩng (rad)
    float yaw;            // Góc quay đầu xe (rad)

    uint8_t checksum;     // XOR checksum của toàn bộ payload
};

// ==========================================
// CẤU TRÚC GÓI DỮ LIỆU ĐIỂM QUÉT LIDAR LD14 (ESP32 -> PC)
// Gói nhị phân raw của LiDAR LD14 / LD06
// ==========================================
#define LD14_HEADER 0x54
#define LD14_VER_LEN 0x2C
#define LD14_POINTS_PER_PACK 12

struct LD14Point {
    uint16_t distance;    // Khoảng cách (mm)
    uint8_t intensity;    // Độ phản xạ / cường độ tín hiệu (0 - 255)
};

struct LD14Packet {
    uint8_t header;       // 0x54
    uint8_t ver_len;      // 0x2C (độ dài 44 byte dữ liệu)
    uint16_t speed;       // Tốc độ quét (độ/giây)
    uint16_t start_angle; // Góc bắt đầu (0.01 độ)
    LD14Point points[LD14_POINTS_PER_PACK]; // 12 điểm đo
    uint16_t end_angle;   // Góc kết thúc (0.01 độ)
    uint16_t timestamp;   // Dấu thời gian (ms)
    uint8_t crc8;         // Checksum CRC-8
};

#pragma pack(pop)

// Hàm tính XOR checksum đơn giản cho các gói tin điều khiển & odom
inline uint8_t calculate_xor_checksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}

#endif // ROBOT_PROTOCOL_H

