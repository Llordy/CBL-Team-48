cmake_minimum_required(VERSION 3.8)
project(gps_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  gps_nav/gps_navigator_node.py
  RENAME gps_navigator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
