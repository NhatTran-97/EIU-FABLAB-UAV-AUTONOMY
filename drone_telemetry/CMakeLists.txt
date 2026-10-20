cmake_minimum_required(VERSION 3.8)
project(drone_telemetry)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# module Python:
#   from drone_telemetry import drone_dialect
ament_python_install_package(${PROJECT_NAME})

# ros2 run drone_telemetry telemetry_node.py
# ros2 run drone_telemetry telemetry_monitor.py --port /dev/ttyUSB0   (monitor phia ground)
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
  # health.py khong phu thuoc ROS
  ament_add_pytest_test(test_health test/test_health.py)
endif()

ament_package()
