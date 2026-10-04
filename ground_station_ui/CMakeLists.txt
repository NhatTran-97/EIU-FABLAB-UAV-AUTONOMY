cmake_minimum_required(VERSION 3.8)
project(ground_station_ui)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# App PySide6/QML -- khong co code C++.
# Cai module python (ca cac goi con core/, qt/): from ground_station_ui.core... import ...
ament_python_install_package(${PROJECT_NAME})

# ros2 run ground_station_ui gcs
install(PROGRAMS ${PROJECT_NAME}/main.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gcs
)

install(DIRECTORY qml config data images
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Tang core: Python thuan, khong can Qt / phan cung.
  ament_add_pytest_test(test_core test/test_core.py)
endif()

ament_package()
