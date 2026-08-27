cmake_minimum_required(VERSION 3.10)
project(msgs_demo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
if(WIN32 AND DEFINED ENV{CONDA_PREFIX})
  set(Python_ROOT_DIR "$ENV{CONDA_PREFIX}")
  set(Python_FIND_REGISTRY NEVER)
  set(Python_FIND_STRATEGY LOCATION)
endif()
find_package(Python REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(WIN32)
  list(GET Python_LIBRARIES 0 PYTHON_LIBRARY)
  set(PYTHON_EXECUTABLE "${Python_EXECUTABLE}" CACHE FILEPATH "" FORCE)
  set(PYTHON_INCLUDE_DIR "${Python_INCLUDE_DIRS}" CACHE PATH "" FORCE)
  set(PYTHON_INCLUDE_DIRS "${Python_INCLUDE_DIRS}" CACHE PATH "" FORCE)
  set(PYTHON_LIBRARY "${PYTHON_LIBRARY}" CACHE FILEPATH "" FORCE)
  set(PYTHON_LIBRARIES "${Python_LIBRARIES}" CACHE STRING "" FORCE)
  set(Python_EXECUTABLE "${Python_EXECUTABLE}" CACHE FILEPATH "" FORCE)
  set(Python_INCLUDE_DIRS "${Python_INCLUDE_DIRS}" CACHE STRING "" FORCE)
  set(Python_LIBRARIES "${Python_LIBRARIES}" CACHE STRING "" FORCE)
  set(Python_NumPy_INCLUDE_DIRS "${Python_NumPy_INCLUDE_DIRS}" CACHE PATH "" FORCE)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Accel.msg"
  "msg/Echos.msg"
  "msg/Imu.msg"
  "msg/LaserScan.msg"
  "msg/Odometry.msg"
  "msg/Point.msg"
  "msg/Pose.msg"
  "msg/PoseStamped.msg"
  "msg/PoseWithCovariance.msg"
  "msg/Power.msg"
  "msg/Quaternion.msg"
  "msg/Twist.msg"
  "msg/TwistWithCovariance.msg"
  "msg/Vector3.msg"
  "srv/AddTwoInts.srv"
  "srv/Empty.srv"
  "srv/GetMap.srv"
  "srv/GetPlan.srv"
  "srv/SetBool.srv"
  "srv/SetCameraInfo.srv"
  "srv/SetMap.srv"
  "srv/TalkerListener.srv"
  "srv/Trigger.srv"
  "action/AddTwoInts.action"
  "action/AutoDocking.action"
  "action/GetMap.action"
  "action/MoveBase.action"
  DEPENDENCIES geometry_msgs nav_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
