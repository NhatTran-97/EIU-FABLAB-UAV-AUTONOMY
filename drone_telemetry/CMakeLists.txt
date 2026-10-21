cmake_minimum_required(VERSION 3.8)
project(drone_telemetry)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python module, importable as:
#   from drone_telemetry import drone_dialect
ament_python_install_package(${PROJECT_NAME})

# ros2 run drone_telemetry telemetry_node.py
# ros2 run drone_telemetry telemetry_monitor.py --port /dev/ttyUSB0   (ground-side monitor)
install(PROGRAMS
  drone_telemetry/telemetry_node.py
  scripts/telemetry_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config mavlink
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # health.py has no ROS dependency
  ament_add_pytest_test(test_health test/test_health.py)
endif()

ament_package()
