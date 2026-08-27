#include <iostream>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("euler_to_quaternion");
  (void)node;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  std::cout << "input roll pitch yaw: ";
  std::cin >> roll >> pitch >> yaw;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  std::cout << "quaternion w=" << q.w() << ", x=" << q.x()
            << ", y=" << q.y() << ", z=" << q.z() << std::endl;

  rclcpp::shutdown();
  return 0;
}
