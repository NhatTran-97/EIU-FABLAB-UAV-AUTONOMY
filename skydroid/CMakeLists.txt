cmake_minimum_required(VERSION 3.8)
project(skydroid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(rtsp_cam src/camera.cpp)

target_include_directories(rtsp_cam PRIVATE ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(rtsp_cam
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(rtsp_cam ${OpenCV_LIBS})



install(TARGETS
    rtsp_cam
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
