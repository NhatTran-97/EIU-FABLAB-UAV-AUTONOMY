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




# add_executable(offboard_node
#   src/offboard_control.cpp
# )


# ament_target_dependencies(offboard_node
#   rclcpp
#   geometry_msgs
#   mavros_msgs

# )


# # Install Python script
# install(PROGRAMS
#   src/nhi.py
#   DESTINATION lib/${PROJECT_NAME}
# )



add_executable(test_mode
    src/test.cpp)

ament_target_dependencies(test_mode
    rclcpp
    mavros_msgs
    geometry_msgs
    custom_msgs
    nav_msgs
)

# pthread_setschedparam (SCHED_FIFO on the setpoint streamer) needs an explicit
# pthread link on older glibc (Jetson Nano / Ubuntu 18.04).
find_package(Threads REQUIRED)
target_link_libraries(test_mode Threads::Threads)

target_include_directories(test_mode
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)    








# target_include_directories(offboard_node
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/offboard_control>
#     $<INSTALL_INTERFACE:include/offboard_control>)

install(TARGETS
 # offboard_node
  test_mode
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch

  DESTINATION share/${PROJECT_NAME})
  



ament_package()
