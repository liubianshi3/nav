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
  scripts/goal_pose_3d_smoke_test.py
  scripts/jt128_dlio_watchdog.py
  scripts/nav_contract_check.py
  scripts/ndt_adapter_dry_run_check.py
  scripts/preflight_check.py
  scripts/task_manager.py
  tools/bootstrap_env.sh
  tools/collect_logs.sh
  tools/configure_real_network.sh
  tools/install_dlio_ros2.sh
  tools/record_jt128_bag.sh
  tools/record_bag.sh
  tools/run_unit_tests.sh
  tools/start_3d_web_console_suite.sh
  tools/start_jt128_3d_closed_loop.sh
  tools/start_jt128_3d_stack.sh
  tools/start_jt128_dlio_mapping.sh
  tools/start_real_stack.sh
  tools/start_real1_suite.sh
  tools/start_web_console_suite.sh
  tools/stop_stack.sh
  tools/stop_jt128_stack.sh
  tools/setup_unitree_dds.sh
  tools/map_quality/check_map_quality.py
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/auto_scan_mission.py
  scripts/config_schema_check.py
  scripts/goal_pose_3d_smoke_test.py
  scripts/jt128_dlio_watchdog.py
  scripts/nav_contract_check.py
  scripts/ndt_adapter_dry_run_check.py
  scripts/task_manager.py
  scripts/nav2_corridor_gate.py
  scripts/traversability_to_obstacle_cloud.py
  scripts/odometry_to_pose_covariance.py
  scripts/pcd_to_2d_map.py
  scripts/a2_battery_publisher.py
  tools/map_quality/check_map_quality.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
