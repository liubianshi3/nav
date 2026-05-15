cmake_minimum_required(VERSION 3.8)
project(a2_control_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(a2_system REQUIRED)
find_package(Threads REQUIRED)

add_executable(a2_control_bridge_node src/a2_control_bridge_node.cpp)
target_compile_features(a2_control_bridge_node PRIVATE cxx_std_17)
target_include_directories(a2_control_bridge_node PRIVATE
  include)
ament_target_dependencies(a2_control_bridge_node
  rclcpp
  geometry_msgs
  std_msgs
  a2_system)

set(UNITREE_SDK2_ROOT "/opt/unitree_robotics" CACHE PATH "Path to unitree_sdk2 checkout")
set(UNITREE_SDK2_CONFIG_HINTS
  "${UNITREE_SDK2_ROOT}/lib/cmake/unitree_sdk2"
  "/opt/unitree_robotics/lib/cmake/unitree_sdk2")
find_package(unitree_sdk2 CONFIG REQUIRED PATHS ${UNITREE_SDK2_CONFIG_HINTS} NO_DEFAULT_PATH)

if(unitree_sdk2_FOUND)
  target_compile_definitions(a2_control_bridge_node PRIVATE A2_ENABLE_UNITREE_SDK=1)
  target_include_directories(a2_control_bridge_node PRIVATE
    "/home/unitree/ZJ/unitree_sdk2-main/include")
  target_link_libraries(a2_control_bridge_node
    unitree_sdk2
    Threads::Threads)
endif()

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
    a2_system
  )
endif()

ament_package()
