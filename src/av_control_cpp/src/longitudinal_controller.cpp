#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "av_carla_interfaces/msg/ego_state.hpp"

#include "av_control_cpp/pid_controller.hpp"

using std::placeholders::_1;

class LongitudinalController : public rclcpp::Node
{
public:
  LongitudinalController()
  : Node("longitudinal_controller"), target_speed_(0.0), current_speed_(0.0)
  {
    declare_parameter<double>("kp", 1.0);
    declare_parameter<double>("ki", 0.1);
    declare_parameter<double>("kd", 0.05);
    declare_parameter<double>("max_output", 1.0);
    declare_parameter<double>("integral_limit", 10.0);
    declare_parameter<double>("target_speed", 0.0);

    target_speed_ = get_parameter("target_speed").as_double();

    pid_.set_gains(
      get_parameter("kp").as_double(),
      get_parameter("ki").as_double(),
      get_parameter("kd").as_double());
    pid_.set_limits(
      get_parameter("max_output").as_double(),
      get_parameter("integral_limit").as_double());

    speed_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/carla/ego_vehicle/speed", 10, std::bind(&LongitudinalController::speed_callback, this, _1));

    ego_state_sub_ = create_subscription<av_carla_interfaces::msg::EgoState>(
      "/ego_state", 10, std::bind(&LongitudinalController::ego_state_callback, this, _1));

    throttle_pub_ = create_publisher<std_msgs::msg::Float64>("/throttle_cmd", 10);
    brake_pub_ = create_publisher<std_msgs::msg::Float64>("/brake_cmd", 10);

    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / 50.0),
      std::bind(&LongitudinalController::control_loop, this));
  }

private:
  void speed_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    current_speed_ = msg->twist.linear.x;
  }

  void ego_state_callback(const av_carla_interfaces::msg::EgoState::SharedPtr msg)
  {
    current_speed_ = msg->speed;
  }

  void control_loop()
  {
    double error = target_speed_ - current_speed_;
    double dt = 0.02;

    double output = pid_.update(error, dt);

    auto throttle_msg = std_msgs::msg::Float64();
    auto brake_msg = std_msgs::msg::Float64();

    if (output >= 0.0) {
      throttle_msg.data = std::min(output, 1.0);
      brake_msg.data = 0.0;
    } else {
      throttle_msg.data = 0.0;
      brake_msg.data = std::min(-output, 1.0);
    }

    throttle_pub_->publish(throttle_msg);
    brake_pub_->publish(brake_msg);
  }

  PIDController pid_;
  double target_speed_;
  double current_speed_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr speed_sub_;
  rclcpp::Subscription<av_carla_interfaces::msg::EgoState>::SharedPtr ego_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LongitudinalController>());
  rclcpp::shutdown();
  return 0;
}

