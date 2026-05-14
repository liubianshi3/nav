cmake_minimum_required(VERSION 3.8)
project(a2_sdk_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(a2_interfaces REQUIRED)
find_package(a2_system REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(a2_sdk_bridge_node src/a2_sdk_bridge_node.cpp)
target_compile_features(a2_sdk_bridge_node PRIVATE cxx_std_17)
ament_target_dependencies(a2_sdk_bridge_node
  rclcpp
  a2_interfaces
  a2_system
  geometry_msgs
  std_msgs)

add_executable(a2_light_bridge_node src/a2_light_bridge_node.cpp)
target_compile_features(a2_light_bridge_node PRIVATE cxx_std_17)
ament_target_dependencies(a2_light_bridge_node
  rclcpp
  a2_interfaces
  a2_system
  std_msgs)

set(UNITREE_SDK2_ROOT "/opt/unitree_robotics" CACHE PATH "Path to unitree_sdk2 checkout")
set(UNITREE_SDK2_CONFIG_HINTS
  "${UNITREE_SDK2_ROOT}/lib/cmake/unitree_sdk2"
  "$ENV{UNITREE_SDK2_ROOT}/lib/cmake/unitree_sdk2"
  "/opt/unitree_robotics/lib/cmake/unitree_sdk2"
  "/usr/local/lib/cmake/unitree_sdk2")
find_package(unitree_sdk2 CONFIG QUIET PATHS ${UNITREE_SDK2_CONFIG_HINTS} NO_DEFAULT_PATH)

if(unitree_sdk2_FOUND)
  target_compile_definitions(a2_sdk_bridge_node PRIVATE A2_ENABLE_UNITREE_SDK=1)
  target_compile_definitions(a2_light_bridge_node PRIVATE A2_ENABLE_UNITREE_SDK=1)
  target_link_libraries(a2_sdk_bridge_node
    unitree_sdk2
    Threads::Threads)
  target_link_libraries(a2_light_bridge_node
    unitree_sdk2
    Threads::Threads)
else()
  set(UNITREE_SDK2_LIB_DIR "${UNITREE_SDK2_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}")
  set(UNITREE_SDK2_THIRDPARTY_DIR "${UNITREE_SDK2_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")

  find_library(UNITREE_SDK2_LIB unitree_sdk2 PATHS "${UNITREE_SDK2_LIB_DIR}" NO_DEFAULT_PATH)
  find_library(UNITREE_DDSC_LIB ddsc PATHS "${UNITREE_SDK2_THIRDPARTY_DIR}" NO_DEFAULT_PATH)
  find_library(UNITREE_DDSCXX_LIB ddscxx PATHS "${UNITREE_SDK2_THIRDPARTY_DIR}" NO_DEFAULT_PATH)

  if(UNITREE_SDK2_LIB AND UNITREE_DDSC_LIB AND UNITREE_DDSCXX_LIB)
    target_compile_definitions(a2_sdk_bridge_node PRIVATE A2_ENABLE_UNITREE_SDK=1)
    target_compile_definitions(a2_light_bridge_node PRIVATE A2_ENABLE_UNITREE_SDK=1)
    target_include_directories(a2_sdk_bridge_node PRIVATE
      "${UNITREE_SDK2_ROOT}/include"
      "${UNITREE_SDK2_ROOT}/thirdparty/include"
      "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx")
    target_include_directories(a2_light_bridge_node PRIVATE
      "${UNITREE_SDK2_ROOT}/include"
      "${UNITREE_SDK2_ROOT}/thirdparty/include"
      "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx")
    target_link_libraries(a2_sdk_bridge_node
      "${UNITREE_SDK2_LIB}"
      "${UNITREE_DDSC_LIB}"
      "${UNITREE_DDSCXX_LIB}"
      Threads::Threads)
    target_link_libraries(a2_light_bridge_node
      "${UNITREE_SDK2_LIB}"
      "${UNITREE_DDSC_LIB}"
      "${UNITREE_DDSCXX_LIB}"
      Threads::Threads)
  else()
    message(WARNING "unitree_sdk2 libraries not found. a2_sdk_bridge_node will build in mock-only mode.")
  endif()
endif()

install(TARGETS a2_sdk_bridge_node a2_light_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
