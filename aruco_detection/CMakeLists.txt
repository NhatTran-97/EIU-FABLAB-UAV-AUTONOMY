cmake_minimum_required(VERSION 3.8)
project(aruco_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# ament_cmake_python la macro dung TRONG package ament_cmake de cai module
# python cho import duoc. Build type van la ament_cmake, khong phai ament_python.
find_package(ament_cmake_python REQUIRED)

# Cai package python aruco_detection/ -> import duoc:
#   from aruco_detection.detector import MarkerDetector
ament_python_install_package(${PROJECT_NAME})

# Node chay duoc: ros2 run aruco_detection aruco_node.py
install(PROGRAMS
  aruco_detection/aruco_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# File calib phai nam trong share/ vi node tim no qua
# get_package_share_directory("aruco_detection").
install(FILES
  aruco_detection/calib_data_mono.json
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
