cmake_minimum_required(VERSION 3.8)
project(a2_control_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(a2_interfaces REQUIRED)
find_package(a2_system REQUIRED)
find_package(a2_unitree_ipc REQUIRED)
find_package(Threads REQUIRED)

add_executable(a2_control_bridge_node src/a2_control_bridge_node.cpp)
target_compile_features(a2_control_bridge_node PRIVATE cxx_std_17)
target_include_directories(a2_control_bridge_node PRIVATE
  include)
ament_target_dependencies(a2_control_bridge_node
  rclcpp
  geometry_msgs
  std_msgs
  a2_interfaces
  a2_system
  a2_unitree_ipc)
target_link_libraries(a2_control_bridge_node Threads::Threads)

install(TARGETS a2_control_bridge_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_control_bridge
    test/test_control_bridge.cpp
    src/a2_control_bridge_node.cpp
  )

  target_compile_definitions(test_control_bridge PRIVATE
    A2_CONTROL_BRIDGE_TEST_BUILD
  )

  target_include_directories(test_control_bridge PRIVATE
    include
  )

  ament_target_dependencies(test_control_bridge
    rclcpp
    geometry_msgs
    std_msgs
    a2_interfaces
    a2_system
    a2_unitree_ipc
  )
endif()

ament_package()
