cmake_minimum_required(VERSION 3.8)
project(a2_control_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(a2_interfaces REQUIRED)
find_package(a2_system REQUIRED)
find_package(Threads REQUIRED)

option(A2_REQUIRE_UNITREE_SDK "Fail configuration when unitree_sdk2 is unavailable" OFF)

add_executable(a2_control_bridge_node src/a2_control_bridge_node.cpp)
target_compile_features(a2_control_bridge_node PRIVATE cxx_std_17)
target_include_directories(a2_control_bridge_node PRIVATE
  include)
ament_target_dependencies(a2_control_bridge_node
  rclcpp
  geometry_msgs
  std_msgs
  a2_interfaces
  a2_system)

set(_A2_DEFAULT_UNITREE_SDK2_ROOT "/opt/unitree_robotics")
if(DEFINED ENV{UNITREE_SDK2_ROOT} AND NOT "$ENV{UNITREE_SDK2_ROOT}" STREQUAL "")
  set(_A2_DEFAULT_UNITREE_SDK2_ROOT "$ENV{UNITREE_SDK2_ROOT}")
elseif(EXISTS "/home/unitree/ZJ/unitree_sdk2-main/include/unitree/robot/a2/sport/sport_client.hpp")
  set(_A2_DEFAULT_UNITREE_SDK2_ROOT "/home/unitree/ZJ/unitree_sdk2-main")
elseif(EXISTS "/home/dell/unitree_sdk2/include/unitree/robot/a2/sport/sport_client.hpp")
  set(_A2_DEFAULT_UNITREE_SDK2_ROOT "/home/dell/unitree_sdk2")
endif()

set(UNITREE_SDK2_ROOT "${_A2_DEFAULT_UNITREE_SDK2_ROOT}" CACHE PATH "Path to unitree_sdk2 checkout")
set(UNITREE_SDK2_CONFIG_HINTS
  "${UNITREE_SDK2_ROOT}/lib/cmake/unitree_sdk2"
  "$ENV{UNITREE_SDK2_ROOT}/lib/cmake/unitree_sdk2"
  "/home/unitree/ZJ/unitree_sdk2-main/lib/cmake/unitree_sdk2"
  "/opt/unitree_robotics/lib/cmake/unitree_sdk2"
  "/unitree/opt/lib/cmake/unitree_sdk2"
  "/usr/local/lib/cmake/unitree_sdk2")
find_package(unitree_sdk2 CONFIG QUIET PATHS ${UNITREE_SDK2_CONFIG_HINTS} NO_DEFAULT_PATH)

if(unitree_sdk2_FOUND)
  target_compile_definitions(a2_control_bridge_node PRIVATE A2_ENABLE_UNITREE_SDK=1)
  target_link_libraries(a2_control_bridge_node
    unitree_sdk2
    Threads::Threads)
else()
  set(UNITREE_SDK2_LIB_DIRS
    "${UNITREE_SDK2_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}"
    "${UNITREE_SDK2_ROOT}/lib/x86_64"
    "${UNITREE_SDK2_ROOT}/lib"
    "${UNITREE_SDK2_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
    "${UNITREE_SDK2_ROOT}/thirdparty/lib/x86_64"
    "${UNITREE_SDK2_ROOT}/thirdparty/lib"
    "/unitree/opt/lib/${CMAKE_SYSTEM_PROCESSOR}"
    "/unitree/opt/lib/x86_64"
    "/unitree/opt/lib")
  set(UNITREE_SDK2_INCLUDE_CANDIDATES
    "${UNITREE_SDK2_ROOT}/include"
    "${UNITREE_SDK2_ROOT}/thirdparty/include"
    "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx"
    "/unitree/opt/include"
    "/unitree/opt/thirdparty/include"
    "/unitree/opt/thirdparty/include/ddscxx")
  set(UNITREE_SDK2_INCLUDE_DIRS "")
  foreach(_include_dir ${UNITREE_SDK2_INCLUDE_CANDIDATES})
    if(EXISTS "${_include_dir}")
      list(APPEND UNITREE_SDK2_INCLUDE_DIRS "${_include_dir}")
    endif()
  endforeach()

  find_library(UNITREE_SDK2_LIB unitree_sdk2 PATHS ${UNITREE_SDK2_LIB_DIRS} NO_DEFAULT_PATH)
  find_library(UNITREE_DDSC_LIB ddsc PATHS ${UNITREE_SDK2_LIB_DIRS} NO_DEFAULT_PATH)
  find_library(UNITREE_DDSCXX_LIB ddscxx PATHS ${UNITREE_SDK2_LIB_DIRS} NO_DEFAULT_PATH)

  if(UNITREE_SDK2_LIB AND UNITREE_DDSC_LIB AND UNITREE_DDSCXX_LIB)
    target_compile_definitions(a2_control_bridge_node PRIVATE A2_ENABLE_UNITREE_SDK=1)
    target_include_directories(a2_control_bridge_node PRIVATE ${UNITREE_SDK2_INCLUDE_DIRS})
    target_link_libraries(a2_control_bridge_node
      "${UNITREE_SDK2_LIB}"
      "${UNITREE_DDSC_LIB}"
      "${UNITREE_DDSCXX_LIB}"
      Threads::Threads)
  else()
    message(WARNING "unitree_sdk2 libraries not found. a2_control_bridge_node will build in mock-only mode.")
    if(A2_REQUIRE_UNITREE_SDK)
      message(FATAL_ERROR "A2_REQUIRE_UNITREE_SDK=ON but unitree_sdk2 is unavailable.")
    endif()
  endif()
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
    a2_interfaces
    a2_system
  )
endif()

ament_package()
