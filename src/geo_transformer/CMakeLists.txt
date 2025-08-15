cmake_minimum_required(VERSION 3.8)
project(geo_transformer)

# Use C++14 if nothing else is specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geo_transformer_interfaces REQUIRED)
# find_package(GeographicLib REQUIRED)

# -----------------------------------------------------------------------------
# Build a library with the core logic (so node and tests can share it)
# -----------------------------------------------------------------------------
add_library(geo_transformer_core src/geo_transformer_node.cpp)
ament_target_dependencies(geo_transformer_core
  rclcpp
  geo_transformer_interfaces
)
target_link_libraries(geo_transformer_core Geographic)  # GeographicLib

# -----------------------------------------------------------------------------
# Build the node executable
# -----------------------------------------------------------------------------
add_executable(geo_transformer_node src/geo_transformer_node.cpp)
target_link_libraries(geo_transformer_node geo_transformer_core)
ament_target_dependencies(geo_transformer_node
  rclcpp
  geo_transformer_interfaces
)

install(TARGETS geo_transformer_node
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_geo_transformer test/test_geo_transformer.cpp)
  target_link_libraries(test_geo_transformer geo_transformer_core)
  ament_target_dependencies(test_geo_transformer
    rclcpp
    geo_transformer_interfaces
  )
endif()

ament_package()
