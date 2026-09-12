cmake_minimum_required(VERSION 3.16)
project(m20_evidence_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)

add_library(evidence_map_core
  src/evidence_map.cpp
)
target_include_directories(evidence_map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(evidence_map_core Eigen3::Eigen ${PCL_LIBRARIES})

add_executable(evidence_mapping_node src/evidence_mapping_node.cpp)
target_include_directories(evidence_mapping_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(evidence_mapping_node evidence_map_core Eigen3::Eigen ${PCL_LIBRARIES})
ament_target_dependencies(evidence_mapping_node
  rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs tf2_ros pcl_conversions
)

install(TARGETS evidence_map_core evidence_mapping_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_evidence_map test/test_evidence_map.cpp)
  target_link_libraries(test_evidence_map evidence_map_core Eigen3::Eigen)
endif()

ament_export_include_directories(include)
ament_export_libraries(evidence_map_core)
ament_package()
