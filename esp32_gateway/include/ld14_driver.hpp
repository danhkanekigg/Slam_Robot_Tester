#ifndef LD14_DRIVER_HPP
#define LD14_DRIVER_HPP

#include <stdint.h>
#include <stddef.h>
#include <HardwareSerial.h>
#include "robot_protocol.h"

/**
 * @brief Driver đọc và phân tích gói tin nhị phân của LiDAR LD14 (tương thích LD06)
 *        Baudrate mặc định: 230400 bps.
 */
class LD14Driver {
public:
    typedef void (*PacketCallback)(const LD14Packet& packet);

    LD14Driver(HardwareSerial* serial_port, int rx_pin, int tx_pin = -1);

    bool begin(uint32_t baudrate = 230400);

    /**
     * @brief Đọc luồng byte từ UART và bóc tách gói tin
     */
    void update();

    /**
     * @brief Đăng ký hàm callback khi nhận được 1 gói quét hợp lệ (12 điểm)
     */
    void registerCallback(PacketCallback cb) { packet_callback_ = cb; }

    /**
     * @brief Tính mã kiểm tra CRC-8 theo thuật toán của nhà sản xuất LD-Robot
     */
    static uint8_t calCRC8(const uint8_t *data, uint16_t length);

private:
    HardwareSerial* serial_;
    int rx_pin_;
    int tx_pin_;
    PacketCallback packet_callback_;

    uint8_t buffer_[sizeof(LD14Packet)];
    size_t buffer_idx_;
};

#endif // LD14_DRIVER_HPP

