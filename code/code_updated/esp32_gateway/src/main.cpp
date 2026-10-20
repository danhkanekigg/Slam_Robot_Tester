#include <Arduino.h>
#include "robot_protocol.h"
#include "ld14_driver.hpp"
#include "wifi_udp_bridge.hpp"

// ==========================================
// CẤU HÌNH THÔNG SỐ MẠNG VÀ CỔNG KẾT NỐI
// ==========================================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";     // Thay bằng tên WiFi của bạn
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"; // Thay bằng mật khẩu WiFi của bạn
const char* TARGET_PC_IP  = "192.168.1.100";      // Địa chỉ IP của máy tính chạy ROS2

const uint16_t LIDAR_UDP_PORT = 8888;
const uint16_t ROBOT_UDP_PORT = 8889;

// ==========================================
// ĐỊNH NGHĨA CHÂN GIAO TIẾP UART
// ==========================================
// UART 1: Kết nối LiDAR LD14 (Baudrate 230400)
#define PIN_LIDAR_RX 16
#define PIN_LIDAR_TX 17
HardwareSerial SerialLidar(1);

// UART 2: Kết nối STM32 (Baudrate 115200)
#define PIN_STM32_RX 18
#define PIN_STM32_TX 19
HardwareSerial SerialSTM32(2);

// Khởi tạo các đối tượng điều khiển
LD14Driver lidar(&SerialLidar, PIN_LIDAR_RX, PIN_LIDAR_TX);
WifiUdpBridge wifi_bridge(WIFI_SSID, WIFI_PASSWORD, TARGET_PC_IP, LIDAR_UDP_PORT, ROBOT_UDP_PORT);

// Callback khi có gói LiDAR hợp lệ từ LD14 -> Gửi ngay qua UDP lên ROS2 PC
void onLidarPacketReceived(const LD14Packet& packet) {
    wifi_bridge.sendLidarPacket(packet);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== KHOI DONG ESP32 COMM GATEWAY ===");

    // Khởi tạo UART kết nối STM32
    SerialSTM32.begin(115200, SERIAL_8N1, PIN_STM32_RX, PIN_STM32_TX);

    // Khởi tạo Wi-Fi và UDP socket
    wifi_bridge.begin();

    // Khởi tạo LiDAR LD14
    lidar.registerCallback(onLidarPacketReceived);
    if (lidar.begin(230400)) {
        Serial.println("Khoi tao LiDAR LD14 thanh cong (230400 bps)!");
    } else {
        Serial.println("Loi khoi tao LiDAR LD14!");
    }

    Serial.println("ESP32 san sang dieu phoi du lieu.");
}

void processSTM32Data() {
    // Nhận gói tin Odometry & IMU từ STM32 qua UART và đẩy lên PC qua UDP
    static uint8_t rx_buffer[sizeof(OdomImuPacket)];
    static size_t rx_idx = 0;

    while (SerialSTM32.available() > 0) {
        uint8_t byte = SerialSTM32.read();

        if (rx_idx == 0) {
            if (byte == (ODOM_IMU_HEADER & 0xFF)) {
                rx_buffer[rx_idx++] = byte;
            }
        } else if (rx_idx == 1) {
            if (byte == ((ODOM_IMU_HEADER >> 8) & 0xFF)) {
                rx_buffer[rx_idx++] = byte;
            } else if (byte == (ODOM_IMU_HEADER & 0xFF)) {
                rx_idx = 1;
            } else {
                rx_idx = 0;
            }
        } else {
            rx_buffer[rx_idx++] = byte;
            if (rx_idx == sizeof(OdomImuPacket)) {
                OdomImuPacket* packet = reinterpret_cast<OdomImuPacket*>(rx_buffer);
                uint8_t expected_cs = calculate_xor_checksum(rx_buffer, sizeof(OdomImuPacket) - 1);

                if (packet->checksum == expected_cs) {
                    // Chuyển tiếp gói Odom/IMU lên máy tính qua UDP
                    wifi_bridge.sendOdomImuPacket(*packet);
                }

                rx_idx = 0;
            }
        }
    }
}

void processIncomingCommands() {
    // Nhận gói tin CmdVel từ máy tính ROS2 qua UDP và chuyển tiếp xuống STM32
    CmdVelPacket cmd;
    if (wifi_bridge.receiveCmdVel(cmd)) {
        // Gửi thẳng qua UART2 xuống STM32
        SerialSTM32.write(reinterpret_cast<const uint8_t*>(&cmd), sizeof(CmdVelPacket));
    }
}

void loop() {
    // 1. Duy trì kết nối Wi-Fi
    wifi_bridge.update();

    // 2. Đọc luồng dữ liệu LiDAR LD14 từ UART1 -> gửi UDP
    lidar.update();

    // 3. Đọc dữ liệu Odometry + IMU từ STM32 -> gửi UDP
    processSTM32Data();

    // 4. Nhận lệnh cmd_vel từ PC -> gửi xuống STM32
    processIncomingCommands();
}

