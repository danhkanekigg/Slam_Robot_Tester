cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  src/robot_udp_bridge_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
if(WIN32)
  target_link_libraries(${PROJECT_NAME}_lib ws2_32)
endif()

add_executable(robot_udp_bridge
  src/main.cpp
)
target_link_libraries(robot_udp_bridge ${PROJECT_NAME}_lib)
ament_target_dependencies(robot_udp_bridge
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
if(WIN32)
  target_link_libraries(robot_udp_bridge ws2_32)
endif()

install(TARGETS
  ${PROJECT_NAME}_lib
  robot_udp_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  urdf
  worlds
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
