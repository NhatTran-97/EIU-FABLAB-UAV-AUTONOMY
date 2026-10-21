cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# STL-50B2 lidar driver (C++): ros2 run obstacle_avoidance stl_50b2_node
add_executable(stl_50b2_node src/stl_50b2_node.cpp)
ament_target_dependencies(stl_50b2_node rclcpp sensor_msgs)
install(TARGETS stl_50b2_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the Python module so it can be imported:
#   from obstacle_avoidance.vfh_plus import VfhPlus
ament_python_install_package(${PROJECT_NAME})

# ROS node: ros2 run obstacle_avoidance avoidance_node.py
install(PROGRAMS
  obstacle_avoidance/avoidance_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Algorithm and frame conversion -- pure numpy, no ROS needed.
  ament_add_pytest_test(test_vfh_plus test/test_vfh_plus.py)
  ament_add_pytest_test(test_scan_processing test/test_scan_processing.py)
  ament_add_pytest_test(test_timing test/test_timing.py)
  ament_add_pytest_test(test_motion_limits test/test_motion_limits.py)
  ament_add_pytest_test(test_closed_loop_dynamics test/test_closed_loop_dynamics.py)
endif()

ament_package()
