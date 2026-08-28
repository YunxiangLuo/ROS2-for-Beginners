#include <memory>
#include <cmath>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "av_carla_interfaces/msg/waypoint_array.hpp"

#include "av_control_cpp/pid_controller.hpp"

using std::placeholders::_1;

class LateralController : public rclcpp::Node
{
public:
  LateralController()
  : Node("lateral_controller")
  {
    declare_parameter<double>("lookahead_distance", 3.0);

    double lookahead = get_parameter("lookahead_distance").as_double();
    pure_pursuit_.set_lookahead_distance(lookahead);

    waypoint_sub_ = create_subscription<av_carla_interfaces::msg::WaypointArray>(
      "/waypoints", 10, std::bind(&LateralController::waypoint_callback, this, _1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego_odom", 10, std::bind(&LateralController::odom_callback, this, _1));

    steering_pub_ = create_publisher<geometry_msgs::msg::Twist>("/steering_cmd", 10);
  }

private:
  void waypoint_callback(const av_carla_interfaces::msg::WaypointArray::SharedPtr msg)
  {
    if (msg->waypoints.empty()) {
      return;
    }
    latest_waypoints_ = msg->waypoints;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (latest_waypoints_.empty()) {
      return;
    }

    current_pose_ = msg->pose.pose.position;

    double roll, pitch, yaw;
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;

    geometry_msgs::msg::Point target = find_lookahead_waypoint();

    double steering = pure_pursuit_.compute_steering(current_pose_, target, current_yaw_);

    auto cmd = geometry_msgs::msg::Twist();
    cmd.angular.z = steering;
    steering_pub_->publish(cmd);
  }

  geometry_msgs::msg::Point find_lookahead_waypoint()
  {
    if (latest_waypoints_.empty()) {
      return current_pose_;
    }

    double lookahead = pure_pursuit_.get_lookahead_distance();
    geometry_msgs::msg::Point best;
    best.x = latest_waypoints_[0].x;
    best.y = latest_waypoints_[0].y;
    best.z = latest_waypoints_[0].z;

    for (const auto & wp : latest_waypoints_) {
      double dx = wp.x - current_pose_.x;
      double dy = wp.y - current_pose_.y;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist >= lookahead) {
        best.x = wp.x;
        best.y = wp.y;
        best.z = wp.z;
        break;
      }
    }

    return best;
  }

  PurePursuitController pure_pursuit_;
  geometry_msgs::msg::Point current_pose_;
  double current_yaw_;
  std::vector<av_carla_interfaces::msg::Waypoint> latest_waypoints_;

  rclcpp::Subscription<av_carla_interfaces::msg::WaypointArray>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr steering_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LateralController>());
  rclcpp::shutdown();
  return 0;
}

