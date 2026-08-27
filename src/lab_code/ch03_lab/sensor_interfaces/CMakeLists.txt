cmake_minimum_required(VERSION 3.8)
project(sensor_interfaces)

find_package(ament_cmake REQUIRED)

# 查找 rosidl_default_generators 包（用于生成消息代码）
find_package(rosidl_default_generators REQUIRED)

# 声明要生成的消息接口文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sensor_data_test test/test_sensor_data.py)
endif()

# ament 打包
ament_package()
