cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_library(teleop SHARED src/teleop.cpp)

target_include_directories(teleop PUBLIC
                          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)
ament_target_dependencies(teleop rclcpp px4_ros2_cpp Eigen3 geometry_msgs std_msgs)


add_executable(teleop_executor
    src/teleopExecutor.cpp
)
target_include_directories(teleop_executor PUBLIC
                          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)
ament_target_dependencies(teleop_executor rclcpp px4_ros2_cpp)

target_link_libraries(teleop_executor teleop)

add_executable(offboard_mode
    src/offboard_mode.cpp)

ament_target_dependencies(offboard_mode
    rclcpp
    mavros_msgs
    geometry_msgs
)

target_include_directories(offboard_mode
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)    





add_executable(custom_mode
    src/custom_mode.cpp)

ament_target_dependencies(custom_mode
    rclcpp
    Eigen3
    px4_ros2_cpp
    geometry_msgs
)

target_include_directories(custom_mode
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)    





install(TARGETS
    teleop
    teleop_executor
    offboard_mode
    custom_mode
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION  lib/${PROJECT_NAME}
)



install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
