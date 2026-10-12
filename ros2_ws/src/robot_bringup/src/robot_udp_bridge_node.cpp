#include "robot_udp_bridge_node.hpp"
#include <cmath>
#include <cstring>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

RobotUdpBridgeNode::RobotUdpBridgeNode()
    : Node("robot_udp_bridge_node"),
      lidar_socket_(INVALID_SOCKET),
      robot_socket_(INVALID_SOCKET),
      is_running_(false),
      scan_ranges_(SCAN_BINS, std::numeric_limits<float>::infinity()),
      scan_intensities_(SCAN_BINS, 0.0f),
      last_packet_end_angle_(0.0f) {

    initParameters();

#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

    // Khởi tạo các Publisher ROS 2
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 50);

    // Khởi tạo TF Broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe topic điều khiển vận tốc xe
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&RobotUdpBridgeNode::cmdVelCallback, this, std::placeholders::_1));

    scan_start_time_ = this->now();

    // Mở các Socket UDP và chạy các luồng nhận dữ liệu
    if (initSockets()) {
        is_running_ = true;
        lidar_thread_ = std::thread(&RobotUdpBridgeNode::lidarReceiveLoop, this);
        robot_thread_ = std::thread(&RobotUdpBridgeNode::robotReceiveLoop, this);
        RCLCPP_INFO(this->get_logger(), "Robot UDP Bridge Node da khoi dong thanh cong!");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Loi khoi tao UDP Sockets!");
    }
}

RobotUdpBridgeNode::~RobotUdpBridgeNode() {
    is_running_ = false;

    if (lidar_thread_.joinable()) lidar_thread_.join();
    if (robot_thread_.joinable()) robot_thread_.join();

    closeSockets();

#ifdef _WIN32
    WSACleanup();
#endif
}

void RobotUdpBridgeNode::initParameters() {
    this->declare_parameter<int>("lidar_port", 8888);
    this->declare_parameter<int>("robot_port", 8889);
    this->declare_parameter<std::string>("esp32_ip", "192.168.1.105");
    this->declare_parameter<int>("esp32_cmd_port", 8889);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<std::string>("laser_frame", "laser_frame");

    lidar_port_     = this->get_parameter("lidar_port").as_int();
    robot_port_     = this->get_parameter("robot_port").as_int();
    esp32_ip_       = this->get_parameter("esp32_ip").as_string();
    esp32_cmd_port_ = this->get_parameter("esp32_cmd_port").as_int();
    odom_frame_     = this->get_parameter("odom_frame").as_string();
    base_frame_     = this->get_parameter("base_frame").as_string();
    laser_frame_    = this->get_parameter("laser_frame").as_string();
}

bool RobotUdpBridgeNode::initSockets() {
    // 1. Tạo socket nhận dữ liệu LiDAR
    lidar_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (lidar_socket_ == INVALID_SOCKET) return false;

    struct sockaddr_in lidar_addr;
    std::memset(&lidar_addr, 0, sizeof(lidar_addr));
    lidar_addr.sin_family = AF_INET;
    lidar_addr.sin_addr.s_addr = INADDR_ANY;
    lidar_addr.sin_port = htons(lidar_port_);

    int opt = 1;
    setsockopt(lidar_socket_, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

    if (bind(lidar_socket_, (struct sockaddr*)&lidar_addr, sizeof(lidar_addr)) == SOCKET_ERROR) {
        RCLCPP_ERROR(this->get_logger(), "Khong the bind UDP Port LiDAR: %d", lidar_port_);
        return false;
    }

    // 2. Tạo socket nhận Odometry/IMU và gửi CmdVel
    robot_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (robot_socket_ == INVALID_SOCKET) return false;

    struct sockaddr_in robot_addr;
    std::memset(&robot_addr, 0, sizeof(robot_addr));
    robot_addr.sin_family = AF_INET;
    robot_addr.sin_addr.s_addr = INADDR_ANY;
    robot_addr.sin_port = htons(robot_port_);

    setsockopt(robot_socket_, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

    if (bind(robot_socket_, (struct sockaddr*)&robot_addr, sizeof(robot_addr)) == SOCKET_ERROR) {
        RCLCPP_ERROR(this->get_logger(), "Khong the bind UDP Port Robot: %d", robot_port_);
        return false;
    }

    // Cài đặt timeout nhận 200ms để luồng nền có thể kiểm tra cờ is_running_ và thoát mượt mà
#ifdef _WIN32
    DWORD timeout_ms = 200;
    setsockopt(lidar_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_ms, sizeof(timeout_ms));
    setsockopt(robot_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_ms, sizeof(timeout_ms));
#else
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000; // 200 ms
    setsockopt(lidar_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
    setsockopt(robot_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#endif

    // Địa chỉ đích gửi lệnh cmd_vel tới ESP32
    std::memset(&esp32_addr_, 0, sizeof(esp32_addr_));
    esp32_addr_.sin_family = AF_INET;
    esp32_addr_.sin_port = htons(esp32_cmd_port_);
    inet_pton(AF_INET, esp32_ip_.c_str(), &esp32_addr_.sin_addr);

    return true;
}

void RobotUdpBridgeNode::closeSockets() {
    if (lidar_socket_ != INVALID_SOCKET) {
        closesocket(lidar_socket_);
        lidar_socket_ = INVALID_SOCKET;
    }
    if (robot_socket_ != INVALID_SOCKET) {
        closesocket(robot_socket_);
        robot_socket_ = INVALID_SOCKET;
    }
}

void RobotUdpBridgeNode::lidarReceiveLoop() {
    uint8_t buffer[sizeof(LD14Packet) + 128];

    while (is_running_) {
        sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);
        int len = recvfrom(lidar_socket_, (char*)buffer, sizeof(buffer), 0,
                           (struct sockaddr*)&sender_addr, &sender_len);

        if (len == sizeof(LD14Packet)) {
            const LD14Packet* packet = reinterpret_cast<const LD14Packet*>(buffer);
            if (packet->header == LD14_HEADER && packet->ver_len == LD14_VER_LEN) {
                processLidarPacket(*packet);
            }
        }
    }
}

void RobotUdpBridgeNode::processLidarPacket(const LD14Packet& packet) {
    float start_deg = packet.start_angle * 0.01f;
    float end_deg = packet.end_angle * 0.01f;

    float angle_diff = end_deg - start_deg;
    if (angle_diff < 0.0f) {
        angle_diff += 360.0f;
    }

    float angle_step = angle_diff / (float)(LD14_POINTS_PER_PACK - 1);

    // Kiểm tra xem đã kết thúc 1 vòng quét 360 độ hay chưa (góc kết thúc nhỏ hơn gói trước)
    if (last_packet_end_angle_ > 0.0f && (end_deg < last_packet_end_angle_ || (last_packet_end_angle_ > 300.0f && end_deg < 50.0f))) {
        publishLaserScan();
        // Reset bộ đệm cho vòng quét mới
        std::fill(scan_ranges_.begin(), scan_ranges_.end(), std::numeric_limits<float>::infinity());
        std::fill(scan_intensities_.begin(), scan_intensities_.end(), 0.0f);
        scan_start_time_ = this->now();
    }
    last_packet_end_angle_ = end_deg;

    // Gán 12 điểm đo vào mảng 360 độ
    for (int i = 0; i < LD14_POINTS_PER_PACK; ++i) {
        float angle = start_deg + (float)i * angle_step;
        if (angle >= 360.0f) angle -= 360.0f;

        int bin_idx = (int)std::floor(angle);
        if (bin_idx >= 0 && bin_idx < SCAN_BINS) {
            float dist_m = (float)packet.points[i].distance / 1000.0f;
            // Lọc các giá trị ngoài dải cảm biến (0.1m đến 12m)
            if (dist_m >= 0.1f && dist_m <= 12.0f) {
                scan_ranges_[bin_idx] = dist_m;
                scan_intensities_[bin_idx] = (float)packet.points[i].intensity;
            }
        }
    }
}

void RobotUdpBridgeNode::publishLaserScan() {
    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = scan_start_time_;
    scan_msg.header.frame_id = laser_frame_;

    scan_msg.angle_min = 0.0f;
    scan_msg.angle_increment = (2.0f * (float)M_PI) / (float)SCAN_BINS;
    scan_msg.angle_max = 2.0f * (float)M_PI - scan_msg.angle_increment;
    scan_msg.time_increment = 0.0f;
    scan_msg.scan_time = 0.1f; // ~10Hz
    scan_msg.range_min = 0.1f;
    scan_msg.range_max = 12.0f;

    scan_msg.ranges = scan_ranges_;
    scan_msg.intensities = scan_intensities_;

    scan_pub_->publish(scan_msg);
}

void RobotUdpBridgeNode::robotReceiveLoop() {
    uint8_t buffer[sizeof(OdomImuPacket) + 128];

    while (is_running_) {
        sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);
        int len = recvfrom(robot_socket_, (char*)buffer, sizeof(buffer), 0,
                           (struct sockaddr*)&sender_addr, &sender_len);

        if (len == sizeof(OdomImuPacket)) {
            const OdomImuPacket* packet = reinterpret_cast<const OdomImuPacket*>(buffer);
            if (packet->header == ODOM_IMU_HEADER) {
                uint8_t cs = calculate_xor_checksum(buffer, sizeof(OdomImuPacket) - 1);
                if (packet->checksum == cs) {
                    processOdomImuPacket(*packet);
                }
            }
        }
    }
}

void RobotUdpBridgeNode::processOdomImuPacket(const OdomImuPacket& packet) {
    rclcpp::Time current_time = this->now();

    // 1. Publish Odometry
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = packet.x;
    odom_msg.pose.pose.position.y = packet.y;
    odom_msg.pose.pose.position.z = 0.0;

    // Quaternion từ yaw: qz = sin(yaw/2), qw = cos(yaw/2)
    float half_yaw = packet.theta / 2.0f;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sinf(half_yaw);
    odom_msg.pose.pose.orientation.w = cosf(half_yaw);

    odom_msg.twist.twist.linear.x = packet.linear_v;
    odom_msg.twist.twist.angular.z = packet.angular_w;

    odom_pub_->publish(odom_msg);

    // 2. Broadcast TF Transform từ odom -> base_footprint
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = packet.x;
    tf_msg.transform.translation.y = packet.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);

    // 3. Publish IMU Data
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = packet.accel_x;
    imu_msg.linear_acceleration.y = packet.accel_y;
    imu_msg.linear_acceleration.z = packet.accel_z;

    imu_msg.angular_velocity.x = packet.gyro_x;
    imu_msg.angular_velocity.y = packet.gyro_y;
    imu_msg.angular_velocity.z = packet.gyro_z;

    imu_msg.orientation = odom_msg.pose.pose.orientation;

    imu_pub_->publish(imu_msg);
}

void RobotUdpBridgeNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    CmdVelPacket packet;
    packet.header = CMD_VEL_HEADER;
    packet.linear_x = (float)msg->linear.x;
    packet.angular_z = (float)msg->angular.z;

    uint8_t* p = reinterpret_cast<uint8_t*>(&packet);
    packet.checksum = calculate_xor_checksum(p, sizeof(CmdVelPacket) - 1);

    if (robot_socket_ != INVALID_SOCKET) {
        sendto(robot_socket_, (const char*)&packet, sizeof(CmdVelPacket), 0,
               (struct sockaddr*)&esp32_addr_, sizeof(esp32_addr_));
    }
}

