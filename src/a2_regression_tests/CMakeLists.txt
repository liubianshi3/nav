cmake_minimum_required(VERSION 3.8)
project(a2_regression_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # Placeholder bag files — CI downloads or skips if absent
  install(DIRECTORY test/bags
    DESTINATION share/${PROJECT_NAME}/test
    FILES_MATCHING PATTERN "*.mcap" PATTERN "*.db3" PATTERN "*.yaml"
  )

  # Regression scenario tests (skip if bags not present)
  ament_add_pytest_test(regression_scenarios test/test_scenarios.py
    TIMEOUT 120
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )

  # Unit-level regression checks (no bags needed)
  ament_add_pytest_test(regression_unit test/test_unit_regressions.py
    TIMEOUT 30
  )
endif()

ament_package()
