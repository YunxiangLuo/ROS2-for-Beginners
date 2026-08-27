#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("coordinate_transformation");
  (void)node;

  tf2::Vector3 v1(1.0, 1.0, 1.0);
  tf2::Vector3 v2(1.0, 0.0, 1.0);

  std::cout << "dot(v1, v2) = " << tf2::tf2Dot(v1, v2) << std::endl;
  std::cout << "length(v2) = " << v2.length() << std::endl;

  tf2::Vector3 v3 = v2.normalized();
  std::cout << "normalized(v2) = (" << v3.x() << ", " << v3.y() << ", " << v3.z() << ")" << std::endl;

  std::cout << "angle(v1, v2) = " << v1.angle(v2) << std::endl;
  std::cout << "distance2(v1, v2) = " << v1.distance2(v2) << std::endl;

  tf2::Vector3 v4 = v1.cross(v2);
  std::cout << "cross(v1, v2) = (" << v4.x() << ", " << v4.y() << ", " << v4.z() << ")" << std::endl;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  std::cout << "quaternion from RPY = (" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << ")" << std::endl;

  tf2::Vector3 axis = q.getAxis();
  std::cout << "axis from quaternion = (" << axis.x() << ", " << axis.y() << ", " << axis.z() << ")" << std::endl;

  tf2::Quaternion q2(axis, 1.570796);
  std::cout << "quaternion from axis/angle = (" << q2.w() << ", " << q2.x() << ", " << q2.y() << ", " << q2.z() << ")" << std::endl;

  tf2::Matrix3x3 matrix(q2);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  matrix.getRPY(roll, pitch, yaw);
  std::cout << "matrix -> RPY = (" << roll << ", " << pitch << ", " << yaw << ")" << std::endl;

  rclcpp::shutdown();
  return 0;
}
