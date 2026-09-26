cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Cai module python de import duoc:
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

  # Thuat toan + doi he toa do -- thuan numpy, chay khong can ROS.
  ament_add_pytest_test(test_vfh_plus test/test_vfh_plus.py)
  ament_add_pytest_test(test_scan_processing test/test_scan_processing.py)
endif()

ament_package()
