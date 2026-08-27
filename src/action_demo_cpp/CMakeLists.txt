cmake_minimum_required(VERSION 3.10)
project(action_demo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(action_demo_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(dishes_server src/dishes_server.cpp)
ament_target_dependencies(dishes_server action_demo_interfaces rclcpp rclcpp_action)

add_executable(dishes_client src/dishes_client.cpp)
ament_target_dependencies(dishes_client action_demo_interfaces rclcpp rclcpp_action)

install(TARGETS
  dishes_server
  dishes_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
