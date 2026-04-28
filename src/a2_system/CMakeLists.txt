cmake_minimum_required(VERSION 3.8)
project(a2_system)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config docs
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/auto_scan_mission.py
  scripts/config_schema_check.py
  scripts/gazebo_outdoor_pipeline.py
  scripts/mock_scan_mission_harness.py
  scripts/nav_contract_check.py
  scripts/preflight_check.py
  scripts/task_manager.py
  tools/bootstrap_env.sh
  tools/collect_logs.sh
  tools/configure_real_network.sh
  tools/record_bag.sh
  tools/run_mock_scan_mission_tests.sh
  tools/run_unit_tests.sh
  tools/run_gazebo_outdoor_pipeline.sh
  tools/start_gazebo_stack.sh
  tools/start_real_stack.sh
  tools/start_real1_suite.sh
  tools/start_web_console_suite.sh
  tools/start_mock_stack.sh
  tools/stop_stack.sh
  tools/setup_unitree_dds.sh
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/auto_scan_mission.py
  scripts/config_schema_check.py
  scripts/mock_scan_mission_harness.py
  scripts/nav_contract_check.py
  scripts/task_manager.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
