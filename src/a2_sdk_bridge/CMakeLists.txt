cmake_minimum_required(VERSION 3.8)
project(a2_sdk_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(a2_interfaces REQUIRED)
find_package(a2_system REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(a2_unitree_ipc REQUIRED)
find_package(Threads REQUIRED)

add_executable(a2_sdk_bridge_node src/a2_sdk_bridge_node.cpp)
target_compile_features(a2_sdk_bridge_node PRIVATE cxx_std_17)
ament_target_dependencies(a2_sdk_bridge_node
  rclcpp
  a2_interfaces
  a2_system
  geometry_msgs
  sensor_msgs
  std_msgs
  a2_unitree_ipc)
target_link_libraries(a2_sdk_bridge_node Threads::Threads)

add_executable(a2_light_bridge_node src/a2_light_bridge_node.cpp)
target_compile_features(a2_light_bridge_node PRIVATE cxx_std_17)
ament_target_dependencies(a2_light_bridge_node
  rclcpp
  a2_interfaces
  a2_system
  std_msgs
  a2_unitree_ipc)
target_link_libraries(a2_light_bridge_node Threads::Threads)

install(TARGETS a2_sdk_bridge_node a2_light_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
