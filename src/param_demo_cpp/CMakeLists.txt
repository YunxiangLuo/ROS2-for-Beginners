cmake_minimum_required(VERSION 3.10)
project(param_demo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(param_demo_cpp_node src/param_demo.cpp)
ament_target_dependencies(param_demo_cpp_node rclcpp)

install(TARGETS
  param_demo_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
