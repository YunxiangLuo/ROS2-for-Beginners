#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

class NameDemoNode : public rclcpp::Node
{
public:
  explicit NameDemoNode(const rclcpp::NodeOptions & options)
  : Node("name_demo", options)
  {
    declare_parameter<int>("serial", -1);
    declare_parameter<int>("global_serial", -1);

    const auto serial = get_parameter("serial").as_int();
    const auto global_serial = get_parameter("global_serial").as_int();

    RCLCPP_INFO(get_logger(), "fully qualified node name: %s", get_fully_qualified_name());
    RCLCPP_INFO(get_logger(), "namespace: %s", get_namespace());
    RCLCPP_INFO_STREAM(get_logger(), "serial = " << serial);
    RCLCPP_INFO_STREAM(get_logger(), "global_serial = " << global_serial);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NameDemoNode>(rclcpp::NodeOptions{});
  (void)node;
  rclcpp::shutdown();
  return 0;
}
