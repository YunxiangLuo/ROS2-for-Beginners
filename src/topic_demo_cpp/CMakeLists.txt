cmake_minimum_required(VERSION 3.10)
project(topic_demo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(topic_demo_interfaces REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp topic_demo_interfaces)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs topic_demo_interfaces)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
