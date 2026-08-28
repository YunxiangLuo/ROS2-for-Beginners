cmake_minimum_required(VERSION 3.8)
project(av_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(av_carla_interfaces REQUIRED)

add_executable(longitudinal_controller src/longitudinal_controller.cpp)
target_include_directories(longitudinal_controller PRIVATE include)
ament_target_dependencies(longitudinal_controller
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
  sensor_msgs
  av_carla_interfaces
)

add_executable(lateral_controller src/lateral_controller.cpp)
target_include_directories(lateral_controller PRIVATE include)
ament_target_dependencies(lateral_controller
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
  sensor_msgs
  av_carla_interfaces
)

add_executable(vehicle_controller src/vehicle_controller.cpp)
target_include_directories(vehicle_controller PRIVATE include)
ament_target_dependencies(vehicle_controller
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
  sensor_msgs
  av_carla_interfaces
)

install(TARGETS
  longitudinal_controller
  lateral_controller
  vehicle_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
