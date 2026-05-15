cmake_minimum_required(VERSION 3.8)
project(a2_state_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(a2_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(a2_state_publisher_node src/a2_state_publisher_node.cpp)
target_compile_features(a2_state_publisher_node PRIVATE cxx_std_17)
ament_target_dependencies(a2_state_publisher_node
  rclcpp
  a2_interfaces
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2)

install(TARGETS a2_state_publisher_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
