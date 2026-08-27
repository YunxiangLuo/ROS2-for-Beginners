#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "service_demo_interfaces/srv/greeting.hpp"

using namespace std::chrono_literals;

class GreetingClient : public rclcpp::Node
{
public:
  GreetingClient()
  : Node("greetings_client")
  {
    client_ = create_client<service_demo_interfaces::srv::Greeting>("greetings");
  }

  int run()
  {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service.");
        return 1;
      }
      RCLCPP_INFO(get_logger(), "Waiting for service...");
    }

    auto request = std::make_shared<service_demo_interfaces::srv::Greeting::Request>();
    request->name = "HAN";
    request->age = 20;

    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to call service greetings");
      return 1;
    }

    RCLCPP_INFO(get_logger(), "Response from server: %s", future.get()->feedback.c_str());
    return 0;
  }

private:
  rclcpp::Client<service_demo_interfaces::srv::Greeting>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GreetingClient>();
  const int rc = node->run();
  rclcpp::shutdown();
  return rc;
}
