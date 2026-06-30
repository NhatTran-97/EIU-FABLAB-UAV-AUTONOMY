cmake_minimum_required(VERSION 3.8)
project(realsense_d435i)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(include)

add_executable(d435i_node
  src/d435i_node.cpp
  src/d435i_driver.cpp
)
ament_target_dependencies(d435i_node rclcpp sensor_msgs)
target_link_libraries(d435i_node realsense2::realsense2)

install(TARGETS d435i_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
