cmake_minimum_required(VERSION 3.8)
project(gimbal_ground_station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# ── ROS 2 ──────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(skydroid_msgs REQUIRED)

# ── Qt 5 ───────────────────────────────────────────────────────────
find_package(Qt5 REQUIRED COMPONENTS
  Core
  Gui
  Qml
  Quick
  QuickControls2
  SerialPort
)

# ── Executable ─────────────────────────────────────────────────────
add_executable(gimbal_ground_station
  src/main.cpp
  src/serial_bridge.cpp
  src/gimbal_controller.cpp
  src/ros_gimbal_state_bridge.cpp
  include/gimbal_ground_station/serial_bridge.hpp
  include/gimbal_ground_station/gimbal_controller.hpp
  include/gimbal_ground_station/ros_gimbal_state_bridge.hpp
  resources.qrc
)

target_include_directories(gimbal_ground_station PRIVATE
  include
)

target_link_libraries(gimbal_ground_station
  Qt5::Core
  Qt5::Gui
  Qt5::Qml
  Qt5::Quick
  Qt5::QuickControls2
  Qt5::SerialPort
)

ament_target_dependencies(gimbal_ground_station
  rclcpp
  skydroid_msgs
)

# ── Install ────────────────────────────────────────────────────────
install(TARGETS gimbal_ground_station
  DESTINATION lib/${PROJECT_NAME}
)

# ── Tests ──────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
