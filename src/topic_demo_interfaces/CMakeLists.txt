cmake_minimum_required(VERSION 3.10)
project(topic_demo_interfaces)

find_package(ament_cmake REQUIRED)
if(WIN32 AND DEFINED ENV{CONDA_PREFIX})
  set(Python_ROOT_DIR "$ENV{CONDA_PREFIX}")
  set(Python_FIND_REGISTRY NEVER)
  set(Python_FIND_STRATEGY LOCATION)
endif()
find_package(Python REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(rosidl_default_generators REQUIRED)

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
  "msg/Gps.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
