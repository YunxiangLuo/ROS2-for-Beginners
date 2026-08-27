#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "service_demo_interfaces/srv/greeting.hpp"

class GreetingServer : public rclcpp::Node
{
public:
  GreetingServer()
  : Node("greetings_server")
  {
    service_ = create_service<service_demo_interfaces::srv::Greeting>(
      "greetings",
      std::bind(
        &GreetingServer::handle_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  void handle_request(
    const std::shared_ptr<service_demo_interfaces::srv::Greeting::Request> request,
    std::shared_ptr<service_demo_interfaces::srv::Greeting::Response> response) const
  {
    RCLCPP_INFO(
      get_logger(), "Request from %s with age %d", request->name.c_str(), request->age);
    response->feedback = "Hi " + request->name + ". I'm server!";
  }

  rclcpp::Service<service_demo_interfaces::srv::Greeting>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GreetingServer>());
  rclcpp::shutdown();
  return 0;
}
