cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(nav_msgs REQUIRED) 




add_executable(offboard_control
    src/offboard_control.cpp
    src/carrot_follower.cpp
    src/setpoint_streamer.cpp
    src/battery_monitor.cpp)

ament_target_dependencies(offboard_control
    rclcpp
    mavros_msgs
    geometry_msgs
    custom_msgs
    nav_msgs
)

# pthread_setschedparam (SCHED_FIFO on the setpoint streamer) needs an explicit
# pthread link on older glibc (Jetson Nano / Ubuntu 18.04).
find_package(Threads REQUIRED)
target_link_libraries(offboard_control Threads::Threads)

target_include_directories(offboard_control
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)    

install(TARGETS
 # offboard_node
  offboard_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch

  DESTINATION share/${PROJECT_NAME})
  



ament_package()
