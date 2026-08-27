#include <memory>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "av_carla_interfaces/msg/ego_state.hpp"
#include "av_carla_interfaces/msg/waypoint_array.hpp"
#include "av_carla_interfaces/msg/control_cmd.hpp"

#include "av_control_cpp/pid_controller.hpp"

using std::placeholders::_1;

class VehicleController : public rclcpp::Node
{
public:
  VehicleController()
  : Node("vehicle_controller"),
    target_speed_(0.0), current_speed_(0.0),
    current_yaw_(0.0)
  {
    declare_parameter<double>("kp", 1.0);
    declare_parameter<double>("ki", 0.1);
    declare_parameter<double>("kd", 0.05);
    declare_parameter<double>("max_output", 1.0);
    declare_parameter<double>("integral_limit", 10.0);
    declare_parameter<double>("lookahead_distance", 3.0);
    declare_parameter<double>("target_speed", 10.0);

    target_speed_ = get_parameter("target_speed").as_double();

    speed_pid_.set_gains(
      get_parameter("kp").as_double(),
      get_parameter("ki").as_double(),
      get_parameter("kd").as_double());
    pure_pursuit_.set_lookahead_distance(get_parameter("lookahead_distance").as_double());

    waypoints_sub_ = create_subscription<av_carla_interfaces::msg::WaypointArray>(
      "/waypoints", 10, std::bind(&VehicleController::plan_callback, this, _1));

    ego_state_sub_ = create_subscription<av_carla_interfaces::msg::EgoState>(
      "/ego_state", 10, std::bind(&VehicleController::ego_state_callback, this, _1));

    control_pub_ = create_publisher<av_carla_interfaces::msg::ControlCmd>(
      "/carla/ego_vehicle/vehicle_control_cmd", 10);

    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / 50.0),
      std::bind(&VehicleController::control_loop, this));
  }

private:
  void plan_callback(const av_carla_interfaces::msg::WaypointArray::SharedPtr msg)
  {
    if (msg->waypoints.empty()) {
      return;
    }
    waypoints_ = msg->waypoints;
  }

  void ego_state_callback(const av_carla_interfaces::msg::EgoState::SharedPtr msg)
  {
    current_speed_ = msg->speed;
    current_pose_ = msg->pose.position;

    double roll, pitch, yaw;
    tf2::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
  }

  void control_loop()
  {
    if (waypoints_.empty()) {
      return;
    }

    double speed_error = target_speed_ - current_speed_;
    double dt = 0.02;
    double throttle_brake = speed_pid_.update(speed_error, dt);

    geometry_msgs::msg::Point target = find_lookahead_waypoint();
    double steering = pure_pursuit_.compute_steering(current_pose_, target, current_yaw_);

    auto cmd = av_carla_interfaces::msg::ControlCmd();
    cmd.throttle = std::max(0.0, std::min(throttle_brake, 1.0));
    cmd.brake = std::max(0.0, std::min(-throttle_brake, 1.0));
    cmd.steer = steering;
    cmd.gear = 1;
    cmd.hand_brake = false;
    cmd.reverse = false;

    control_pub_->publish(cmd);
  }

  geometry_msgs::msg::Point find_lookahead_waypoint()
  {
    if (waypoints_.empty()) {
      return current_pose_;
    }

    double lookahead = pure_pursuit_.get_lookahead_distance();
    geometry_msgs::msg::Point best;
    best.x = waypoints_[0].x;
    best.y = waypoints_[0].y;
    best.z = waypoints_[0].z;

    for (const auto & wp : waypoints_) {
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

  PIDController speed_pid_;
  PurePursuitController pure_pursuit_;

  double target_speed_;
  double current_speed_;
  geometry_msgs::msg::Point current_pose_;
  double current_yaw_;
  std::vector<av_carla_interfaces::msg::Waypoint> waypoints_;

  rclcpp::Subscription<av_carla_interfaces::msg::WaypointArray>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<av_carla_interfaces::msg::EgoState>::SharedPtr ego_state_sub_;
  rclcpp::Publisher<av_carla_interfaces::msg::VehicleControl>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleController>());
  rclcpp::shutdown();
  return 0;
}

