cmake_minimum_required(VERSION 3.8)
project(gps_nav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GpsGoal.action"
)

ament_package()
