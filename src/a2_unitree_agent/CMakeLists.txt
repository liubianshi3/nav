cmake_minimum_required(VERSION 3.8)
project(a2_unitree_agent)

find_package(ament_cmake REQUIRED)
find_package(a2_unitree_ipc REQUIRED)
find_package(Threads REQUIRED)

option(A2_REQUIRE_UNITREE_SDK "Fail configuration when unitree_sdk2 is unavailable" OFF)

add_executable(unitree_agent src/unitree_agent.cpp)
target_compile_features(unitree_agent PRIVATE cxx_std_17)
ament_target_dependencies(unitree_agent a2_unitree_ipc)
target_link_libraries(unitree_agent Threads::Threads)

set(UNITREE_SDK2_ROOT "/opt/unitree_robotics" CACHE PATH "Path to unitree_sdk2 checkout")
set(UNITREE_SDK2_CONFIG_HINTS
  "${UNITREE_SDK2_ROOT}/lib/cmake/unitree_sdk2"
  "$ENV{UNITREE_SDK2_ROOT}/lib/cmake/unitree_sdk2"
  "/opt/unitree_robotics/lib/cmake/unitree_sdk2"
  "/unitree/opt/lib/cmake/unitree_sdk2"
  "/usr/local/lib/cmake/unitree_sdk2")
find_package(unitree_sdk2 CONFIG QUIET PATHS ${UNITREE_SDK2_CONFIG_HINTS} NO_DEFAULT_PATH)

if(unitree_sdk2_FOUND)
  target_compile_definitions(unitree_agent PRIVATE A2_ENABLE_UNITREE_SDK=1)
  target_link_libraries(unitree_agent unitree_sdk2)
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
    target_compile_definitions(unitree_agent PRIVATE A2_ENABLE_UNITREE_SDK=1)
    target_include_directories(unitree_agent PRIVATE ${UNITREE_SDK2_INCLUDE_DIRS})
    target_link_libraries(unitree_agent
      "${UNITREE_SDK2_LIB}"
      "${UNITREE_DDSC_LIB}"
      "${UNITREE_DDSCXX_LIB}")
  else()
    message(WARNING "unitree_sdk2 libraries not found. unitree_agent will build in mock-only mode.")
    if(A2_REQUIRE_UNITREE_SDK)
      message(FATAL_ERROR "A2_REQUIRE_UNITREE_SDK=ON but unitree_sdk2 is unavailable.")
    endif()
  endif()
endif()

install(TARGETS unitree_agent
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/start_unitree_agent.sh
  DESTINATION share/${PROJECT_NAME}/scripts)
install(FILES systemd/a2-unitree-agent.service
  DESTINATION share/${PROJECT_NAME}/systemd)

ament_package()
