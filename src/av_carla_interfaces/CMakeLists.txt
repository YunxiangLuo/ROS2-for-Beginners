cmake_minimum_required(VERSION 3.8)
project(av_carla_interfaces)

if(COMMAND cargo_check_for_android)
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/ControlCmd.msg"
  "msg/PerceptionObject.msg"
  "msg/PerceptionObjectArray.msg"
  "msg/Waypoint.msg"
  "msg/WaypointArray.msg"
  "msg/EgoState.msg"
  "msg/Cluster.msg"
  "msg/ClusterArray.msg"
  "msg/CollisionEvent.msg"
)

set(srv_files
  "srv/PlanPath.srv"
)

set(action_files
  "action/Navigate.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

