#ifndef ROBOT_UDP_BRIDGE_NODE_HPP
#define ROBOT_UDP_BRIDGE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <atomic>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  typedef int socklen_t;
#else
  #include <sys/types.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #include <fcntl.h>
  #define SOCKET int
  #define INVALID_SOCKET -1
  #define SOCKET_ERROR -1
  #define closesocket close
#endif

#include "robot_protocol.h"

/**
 * @brief Node ROS 2 C++ cầu nối UDP giữa máy tính PC và ESP32
 *        - Nhận dữ liệu LiDAR LD14 từ ESP32 qua UDP (port 8888) -> Publish /scan
 *        - Nhận dữ liệu Odometry & IMU từ ESP32 qua UDP (port 8889) -> Publish /odom, /imu/data, và TF
 *        - Subscribe /cmd_vel -> Đóng gói và gửi UDP tới ESP32 để điều khiển STM32
 */
class RobotUdpBridgeNode : public rclcpp::Node {
public:
    RobotUdpBridgeNode();
    virtual ~RobotUdpBridgeNode();

private:
    void initParameters();
    bool initSockets();
    void closeSockets();

    // Luồng nền lắng nghe socket
    void lidarReceiveLoop();
    void robotReceiveLoop();

    // Xử lý gói tin
    void processLidarPacket(const LD14Packet& packet);
    void publishLaserScan();
    void processOdomImuPacket(const OdomImuPacket& packet);

    // Callback nhận lệnh vận tốc từ ROS2
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Thông số cấu hình
    int lidar_port_;
    int robot_port_;
    std::string esp32_ip_;
    int esp32_cmd_port_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string laser_frame_;

    // ROS 2 Publishers & Subscribers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Socket handles
    SOCKET lidar_socket_;
    SOCKET robot_socket_;
    struct sockaddr_in esp32_addr_;

    // Threads
    std::atomic<bool> is_running_;
    std::thread lidar_thread_;
    std::thread robot_thread_;

    // Bộ đệm tích luỹ dữ liệu 1 vòng quét 360 độ của LD14
    static constexpr int SCAN_BINS = 360; // 360 bins (mỗi bin 1 độ)
    std::vector<float> scan_ranges_;
    std::vector<float> scan_intensities_;
    float last_packet_end_angle_;
    rclcpp::Time scan_start_time_;
};

#endif // ROBOT_UDP_BRIDGE_NODE_HPP

