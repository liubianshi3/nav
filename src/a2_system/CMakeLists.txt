cmake_minimum_required(VERSION 3.8)
project(a2_system)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config docs
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/gazebo_outdoor_pipeline.py
  scripts/nav_contract_check.py
  scripts/preflight_check.py
  tools/bootstrap_env.sh
  tools/collect_logs.sh
  tools/configure_real_network.sh
  tools/record_bag.sh
  tools/run_gazebo_outdoor_pipeline.sh
  tools/start_gazebo_stack.sh
  tools/start_real_stack.sh
  tools/start_real1_suite.sh
  tools/start_mock_stack.sh
  tools/stop_stack.sh
  tools/setup_unitree_dds.sh
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/nav_contract_check.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
