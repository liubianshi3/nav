cmake_minimum_required(VERSION 3.8)
project(a2_system)

find_package(ament_cmake REQUIRED)
find_package(octomap REQUIRED)

add_executable(octomap_to_2d_grid_cpp src/octomap_to_2d_grid.cpp)
target_link_libraries(octomap_to_2d_grid_cpp ${OCTOMAP_LIBRARIES})
target_include_directories(octomap_to_2d_grid_cpp PRIVATE ${OCTOMAP_INCLUDE_DIRS})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config docs
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/abc_isolation_eval.py
  scripts/auto_scan_mission.py
  scripts/config_schema_check.py
  scripts/append_3d_test_record.py
  scripts/goal_pose_3d_smoke_test.py
  scripts/industrial_3d_closed_loop_check.py
  scripts/industrial_3d_nav_preflight.py
  scripts/jt128_dlio_watchdog.py
  scripts/nav_contract_check.py
  scripts/ndt_adapter_dry_run_check.py
  scripts/odometry_tf_broadcaster.py
  scripts/preflight_check.py
  scripts/sensor_covariance_injector.py
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
  scripts/abc_isolation_eval.py
  scripts/auto_scan_mission.py
  scripts/config_schema_check.py
  scripts/append_3d_test_record.py
  scripts/goal_pose_3d_smoke_test.py
  scripts/industrial_3d_closed_loop_check.py
  scripts/industrial_3d_nav_preflight.py
  scripts/jt128_dlio_watchdog.py
  scripts/nav_contract_check.py
  scripts/ndt_adapter_dry_run_check.py
  scripts/odometry_tf_broadcaster.py
  scripts/octomap_mapping_node.py
  scripts/octomap_to_2d_grid.py
  scripts/sensor_covariance_injector.py
  scripts/task_manager.py
  scripts/traversability_to_obstacle_cloud.py
  scripts/odometry_to_pose_covariance.py
  scripts/pcd_to_2d_map.py
  scripts/a2_battery_publisher.py
  tools/map_quality/check_map_quality.py
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  octomap_to_2d_grid_cpp
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
