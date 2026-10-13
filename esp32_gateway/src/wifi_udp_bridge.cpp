#include "wifi_udp_bridge.hpp"

WifiUdpBridge::WifiUdpBridge(const char* ssid, const char* password,
                             const char* target_pc_ip,
                             uint16_t lidar_port,
                             uint16_t robot_port)
    : ssid_(ssid),
      password_(password),
      lidar_port_(lidar_port),
      robot_port_(robot_port),
      last_reconnect_check_ms_(0) {
    pc_ip_.fromString(target_pc_ip);
}

bool WifiUdpBridge::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid_, password_);

    Serial.print("Dang ket noi WiFi: ");
    Serial.println(ssid_);

    uint32_t start_attempt = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start_attempt < 10000)) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("WiFi ket noi thanh cong! IP ESP32: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Chua the ket noi WiFi, se thu lai trong loop.");
    }

    // Khởi tạo UDP listener trên port điều khiển robot
    udp_robot_.begin(robot_port_);
    return true;
}

void WifiUdpBridge::update() {
    uint32_t now = millis();
    if (now - last_reconnect_check_ms_ > 5000) {
        last_reconnect_check_ms_ = now;
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Mat ket noi WiFi, dang ket noi lai...");
            WiFi.reconnect();
        }
    }
}

bool WifiUdpBridge::isConnected() const {
    return WiFi.status() == WL_CONNECTED;
}

void WifiUdpBridge::sendLidarPacket(const LD14Packet& packet) {
    if (!isConnected()) return;

    udp_lidar_.beginPacket(pc_ip_, lidar_port_);
    udp_lidar_.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(LD14Packet));
    udp_lidar_.endPacket();
}

void WifiUdpBridge::sendOdomImuPacket(const OdomImuPacket& packet) {
    if (!isConnected()) return;

    udp_robot_.beginPacket(pc_ip_, robot_port_);
    udp_robot_.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(OdomImuPacket));
    udp_robot_.endPacket();
}

bool WifiUdpBridge::receiveCmdVel(CmdVelPacket& cmd_out) {
    int packet_size = udp_robot_.parsePacket();
    if (packet_size >= (int)sizeof(CmdVelPacket)) {
        uint8_t buffer[sizeof(CmdVelPacket)];
        int len = udp_robot_.read(buffer, sizeof(CmdVelPacket));

        if (len == sizeof(CmdVelPacket)) {
            CmdVelPacket* packet = reinterpret_cast<CmdVelPacket*>(buffer);
            if (packet->header == CMD_VEL_HEADER) {
                uint8_t cs = calculate_xor_checksum(buffer, sizeof(CmdVelPacket) - 1);
                if (packet->checksum == cs) {
                    cmd_out = *packet;
                    udp_robot_.flush();
                    return true;
                }
            }
        }
        udp_robot_.flush();
    }
    return false;
}

