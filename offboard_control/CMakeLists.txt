cmake_minimum_required(VERSION 3.8)
project(offboard_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python package imported by mission_manager.py: plan compilation, steps, navigation strategies.
ament_python_install_package(mission_pipeline PACKAGE_DIR px4_offboard/mission_pipeline)

install(PROGRAMS
  px4_offboard/mission_manager.py
  px4_offboard/offboard_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Pure Python, no ROS needed.
  ament_add_pytest_test(test_mission_pipeline test/test_mission_pipeline.py)
  # Real nodes with intercepted publishers; skipped when the ROS messages are not available.
  ament_add_pytest_test(test_mission_manager_node test/test_mission_manager_node.py)
  ament_add_pytest_test(test_offboard_control_node test/test_offboard_control_node.py)
endif()

ament_package()
