cmake_minimum_required(VERSION 3.16)
project(m20_drdds_evidence LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(drdds REQUIRED CONFIG PATHS /usr/local/lib/cmake/drdds NO_DEFAULT_PATH)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

add_executable(drdds_evidence_node
  drdds_evidence_node.cpp
  ../src/evidence_map.cpp
)
target_include_directories(drdds_evidence_node PRIVATE
  ../include
  ${drdds_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(drdds_evidence_node PRIVATE
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${drdds_LIBRARIES}
)
target_compile_options(drdds_evidence_node PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(drdds_evidence_node PROPERTIES
  BUILD_RPATH /usr/local/lib
  INSTALL_RPATH /usr/local/lib
)
