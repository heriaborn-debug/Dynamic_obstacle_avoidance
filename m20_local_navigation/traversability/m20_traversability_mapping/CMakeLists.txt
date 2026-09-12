cmake_minimum_required(VERSION 3.16)
project(m20_traversability_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

add_library(traversability_core src/traversability_grid.cpp)
target_include_directories(traversability_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(traversability_core Eigen3::Eigen)

add_executable(traversability_mapping_node src/traversability_mapping_node.cpp)
target_include_directories(traversability_mapping_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(traversability_mapping_node traversability_core Eigen3::Eigen ${PCL_LIBRARIES})
ament_target_dependencies(traversability_mapping_node
  rclcpp sensor_msgs nav_msgs diagnostic_msgs pcl_conversions)

install(TARGETS traversability_core traversability_mapping_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_traversability_grid test/test_traversability_grid.cpp)
  target_link_libraries(test_traversability_grid traversability_core Eigen3::Eigen)
endif()

ament_export_include_directories(include)
ament_export_libraries(traversability_core)
ament_package()
