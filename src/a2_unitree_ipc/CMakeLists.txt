cmake_minimum_required(VERSION 3.8)
project(a2_unitree_ipc)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

if(Protobuf_INCLUDE_DIRS)
  list(GET Protobuf_INCLUDE_DIRS 0 PROTOBUF_FIRST_INCLUDE_DIR)
  get_filename_component(PROTOBUF_PREFIX "${PROTOBUF_FIRST_INCLUDE_DIR}" DIRECTORY)
  find_program(PROTOC_FROM_PROTOBUF_PREFIX protoc HINTS "${PROTOBUF_PREFIX}/bin" NO_DEFAULT_PATH)
  if(PROTOC_FROM_PROTOBUF_PREFIX)
    set(Protobuf_PROTOC_EXECUTABLE "${PROTOC_FROM_PROTOBUF_PREFIX}")
  endif()
endif()

set(PROTO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../proto")
set(UNITREE_AGENT_PROTO "${PROTO_ROOT}/a2/unitree_agent.proto")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(UNITREE_AGENT_PROTO_SRCS "${PROTO_GEN_DIR}/a2/unitree_agent.pb.cc")
set(UNITREE_AGENT_PROTO_HDRS "${PROTO_GEN_DIR}/a2/unitree_agent.pb.h")

add_custom_command(
  OUTPUT
    "${UNITREE_AGENT_PROTO_SRCS}"
    "${UNITREE_AGENT_PROTO_HDRS}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${PROTO_GEN_DIR}/a2"
  COMMAND "${Protobuf_PROTOC_EXECUTABLE}" --cpp_out="${PROTO_GEN_DIR}" -I "${PROTO_ROOT}" "${UNITREE_AGENT_PROTO}"
  DEPENDS "${UNITREE_AGENT_PROTO}"
  COMMENT "Generating protobuf bindings for unitree_agent IPC"
  VERBATIM
)

add_library(${PROJECT_NAME}
  ${UNITREE_AGENT_PROTO_SRCS}
  src/client.cpp
  src/protocol.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROTO_GEN_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    protobuf::libprotobuf
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Protobuf)
ament_package()
