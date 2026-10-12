#ifndef WIFI_UDP_BRIDGE_HPP
#define WIFI_UDP_BRIDGE_HPP

#include <WiFi.h>
#include <WiFiUdp.h>
#include "robot_protocol.h"

/**
 * @brief Lớp quản lý kết nối Wi-Fi và truyền phát dữ liệu qua giao thức UDP 2 chiều
 *        - Truyền gói LiDAR LD14 tới PC (Port 8888)
 *        - Truyền gói Odometry/IMU tới PC (Port 8889)
 *        - Nhận gói lệnh điều khiển CmdVel từ PC (Port 8889)
 */
class WifiUdpBridge {
public:
    WifiUdpBridge(const char* ssid, const char* password,
                  const char* target_pc_ip,
                  uint16_t lidar_port = 8888,
                  uint16_t robot_port = 8889);

    bool begin();

    /**
     * @brief Duy trì kết nối Wi-Fi (tự động kết nối lại nếu rớt mạng)
     */
    void update();

    bool isConnected() const;

    /**
     * @brief Gửi gói LiDAR LD14 lên máy tính PC
     */
    void sendLidarPacket(const LD14Packet& packet);

    /**
     * @brief Gửi gói Odometry & IMU lên máy tính PC
     */
    void sendOdomImuPacket(const OdomImuPacket& packet);

    /**
     * @brief Kiểm tra và nhận gói tin CmdVel từ PC gửi xuống
     * @param cmd_out Dữ liệu vận tốc nhận được
     * @return true nếu có gói tin mới hợp lệ
     */
    bool receiveCmdVel(CmdVelPacket& cmd_out);

private:
    const char* ssid_;
    const char* password_;
    IPAddress pc_ip_;
    uint16_t lidar_port_;
    uint16_t robot_port_;

    WiFiUDP udp_lidar_;
    WiFiUDP udp_robot_;

    uint32_t last_reconnect_check_ms_;
};

#endif // WIFI_UDP_BRIDGE_HPP

