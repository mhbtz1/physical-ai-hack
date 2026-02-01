cmake_minimum_required(VERSION 3.8)
project(minimal_node_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(minimal_node minimal_node.cpp)
ament_target_dependencies(minimal_node rclcpp)

install(TARGETS
  minimal_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()