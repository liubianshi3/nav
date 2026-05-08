cmake_minimum_required(VERSION 3.14)
project(a2_ground_segmentation_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(A2_NATIVE_OPT "Enable -O3 -march=native (host-pinned binary)" ON)
if(A2_NATIVE_OPT AND CMAKE_BUILD_TYPE MATCHES "Release|RelWithDebInfo|")
  add_compile_options(-O3 -march=native)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

add_library(${PROJECT_NAME}_core
  src/ground_segmenter.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_core PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC A2_GS_HAVE_OPENMP=1)
endif()

add_executable(ground_segmentation_cpp_node
  src/ground_segmentation_cpp_node.cpp
)
target_link_libraries(ground_segmentation_cpp_node ${PROJECT_NAME}_core)
ament_target_dependencies(ground_segmentation_cpp_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  builtin_interfaces
)

install(TARGETS
  ${PROJECT_NAME}_core
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS ground_segmentation_cpp_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ground_segmenter test/test_ground_segmenter.cpp)
  target_link_libraries(test_ground_segmenter ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rclcpp sensor_msgs nav_msgs std_msgs builtin_interfaces Eigen3)

ament_package()
