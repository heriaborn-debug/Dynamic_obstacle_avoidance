cmake_minimum_required(VERSION 3.8)
project(m20_first_planner_v2)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(
  m20_first_planner_v2_cpp
  src/m20_first_planner_v2_cpp.cpp
)
ament_target_dependencies(
  m20_first_planner_v2_cpp
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

install(
  TARGETS
    m20_first_planner_v2_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch_nx/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
