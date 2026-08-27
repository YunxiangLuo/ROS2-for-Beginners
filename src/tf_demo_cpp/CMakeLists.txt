cmake_minimum_required(VERSION 3.10)
project(tf_demo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(euler_to_quaternion src/euler_to_quaternion.cpp)
ament_target_dependencies(euler_to_quaternion geometry_msgs rclcpp tf2)

add_executable(quaternion_to_euler src/quaternion_to_euler.cpp)
ament_target_dependencies(quaternion_to_euler geometry_msgs rclcpp tf2)

add_executable(coordinate_transformation src/coordinate_transformation.cpp)
ament_target_dependencies(coordinate_transformation geometry_msgs rclcpp tf2)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster geometry_msgs rclcpp tf2 tf2_ros)

add_executable(tf_listener src/tf_listener.cpp)
ament_target_dependencies(tf_listener geometry_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS
  coordinate_transformation
  euler_to_quaternion
  quaternion_to_euler
  tf_broadcaster
  tf_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
