cmake_minimum_required(VERSION 3.8)
project(aruco_detection)

find_package(ament_cmake REQUIRED)
# Macro dung BEN TRONG mot package ament_cmake de cai module python cho
# import duoc. Build type van la ament_cmake.
find_package(ament_cmake_python REQUIRED)

# Cai thu muc aruco_detection/ de dong nay chay duoc:
#   from aruco_detection.detector import MarkerDetector
ament_python_install_package(${PROJECT_NAME})

# ROS node: ros2 run aruco_detection <ten>
install(PROGRAMS
  aruco_detection/aruco_node.py
  aruco_detection/marker_localizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Cong cu chay tay (python3 <file>), khong phai ROS node.
install(PROGRAMS
  scripts/aruco_generator.py
  scripts/aruco_pose_estimation.py
  scripts/fractal_test.py
  scripts/vector_field_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

# File hieu chuan phai nam trong share/ vi node tim no qua
# get_package_share_directory("aruco_detection").
install(FILES
  aruco_detection/calib_data_mono.json
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Toan hoc he toa do -- thuan numpy, chay khong can ROS.
  ament_add_pytest_test(test_geometry test/test_geometry.py)
endif()

ament_package()
