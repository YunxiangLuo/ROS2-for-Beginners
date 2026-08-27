#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "topic_demo_interfaces/msg/gps.hpp"

class GpsListener : public rclcpp::Node
{
public:
  GpsListener()
  : Node("listener")
  {
    subscription_ = create_subscription<topic_demo_interfaces::msg::Gps>(
      "gps_info", 10,
      std::bind(&GpsListener::handle_message, this, std::placeholders::_1));
  }

private:
  void handle_message(const topic_demo_interfaces::msg::Gps::SharedPtr msg) const
  {
    const float distance = std::sqrt((msg->x * msg->x) + (msg->y * msg->y));
    RCLCPP_INFO(
      get_logger(),
      "Listener: Distance to origin = %.6f, state: %s",
      distance,
      msg->state.c_str());
  }

  rclcpp::Subscription<topic_demo_interfaces::msg::Gps>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsListener>());
  rclcpp::shutdown();
  return 0;
}
