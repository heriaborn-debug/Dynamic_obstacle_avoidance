cmake_minimum_required(VERSION 3.8)
project(m20_navigation_integration)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/m20_astar_global_planner.py
  scripts/m20_path_lookahead.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
