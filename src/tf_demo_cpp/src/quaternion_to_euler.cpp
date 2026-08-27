#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("quaternion_to_euler");
  (void)node;

  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  std::cout << "input quaternion w x y z: ";
  std::cin >> w >> x >> y >> z;

  tf2::Quaternion q(x, y, z, w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  std::cout << "euler roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw << std::endl;

  rclcpp::shutdown();
  return 0;
}
