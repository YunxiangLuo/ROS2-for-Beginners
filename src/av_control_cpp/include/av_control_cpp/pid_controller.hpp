#ifndef AV_CONTROL_CPP__PID_CONTROLLER_HPP_
#define AV_CONTROL_CPP__PID_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"

class PIDController
{
public:
  PIDController()
  : kp_(0.0), ki_(0.0), kd_(0.0),
    max_output_(0.0), integral_limit_(0.0),
    integral_(0.0), prev_error_(0.0)
  {}

  PIDController(double kp, double ki, double kd, double max_output, double integral_limit)
  : kp_(kp), ki_(ki), kd_(kd),
    max_output_(max_output), integral_limit_(integral_limit),
    integral_(0.0), prev_error_(0.0)
  {}

  void reset()
  {
    integral_ = 0.0;
    prev_error_ = 0.0;
  }

  double update(double error, double dt)
  {
    if (dt <= 0.0) {
      return 0.0;
    }

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);

    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    output = std::clamp(output, -max_output_, max_output_);

    if (std::abs(output) >= max_output_) {
      integral_ -= error * dt;
      integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    }

    return output;
  }

  void set_gains(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void set_limits(double max_output, double integral_limit)
  {
    max_output_ = std::max(0.0, max_output);
    integral_limit_ = std::max(0.0, integral_limit);
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
  }

  void get_gains(double & kp, double & ki, double & kd) const
  {
    kp = kp_;
    ki = ki_;
    kd = kd_;
  }

private:
  double kp_;
  double ki_;
  double kd_;
  double max_output_;
  double integral_limit_;
  double integral_;
  double prev_error_;
};

class PurePursuitController
{
public:
  PurePursuitController() : lookahead_distance_(3.0) {}

  explicit PurePursuitController(double lookahead_distance)
  : lookahead_distance_(lookahead_distance)
  {}

  double compute_steering(
    const geometry_msgs::msg::Point & current_position,
    const geometry_msgs::msg::Point & target_waypoint,
    double current_yaw)
  {
    double dx = target_waypoint.x - current_position.x;
    double dy = target_waypoint.y - current_position.y;

    double alpha = std::atan2(dy, dx) - current_yaw;
    alpha = std::atan2(std::sin(alpha), std::cos(alpha));

    double lookahead = std::sqrt(dx * dx + dy * dy);
    if (lookahead < 0.001) {
      return 0.0;
    }

    double steering = std::atan2(2.0 * 1.5 * std::sin(alpha), lookahead);
    return std::clamp(steering, -1.0, 1.0);
  }

  void set_lookahead_distance(double distance)
  {
    lookahead_distance_ = distance;
  }

  double get_lookahead_distance() const
  {
    return lookahead_distance_;
  }

private:
  double lookahead_distance_;
};

#endif  // AV_CONTROL_CPP__PID_CONTROLLER_HPP_
