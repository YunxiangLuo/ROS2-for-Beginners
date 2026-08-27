#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamDemoNode : public rclcpp::Node
{
public:
  ParamDemoNode()
  : Node("param_demo_cpp"), done_(false), loop_count_(0)
  {
    declare_parameter<int>("param1", 111);
    declare_parameter<int>("param2", 222);
    declare_parameter<int>("param3", 33333);
    declare_parameter<int>("param4", 0);
    declare_parameter<int>("param5", 0);
    declare_parameter<int>("max_loops", 3);

    log_parameter("param1");
    log_parameter("param2");
    log_parameter("param3");

    set_parameter(rclcpp::Parameter("param4", 4));
    set_parameter(rclcpp::Parameter("param5", 5));
    RCLCPP_INFO(get_logger(), "Param4 is set to be 4");
    RCLCPP_INFO(get_logger(), "Param5 is set to be 5");

    timer_ = create_wall_timer(1s, std::bind(&ParamDemoNode::tick, this));
  }

  bool done() const
  {
    return done_;
  }

private:
  void log_parameter(const std::string & name)
  {
    const int value = get_parameter(name).as_int();
    RCLCPP_INFO(get_logger(), "Get %s = %d", name.c_str(), value);
  }

  void tick()
  {
    ++loop_count_;
    const int max_loops = get_parameter("max_loops").as_int();

    if (loop_count_ == 1) {
      set_parameter(rclcpp::Parameter("param2", 2));
      RCLCPP_INFO(get_logger(), "Param2 updated to 2");
    }

    std::vector<std::string> names = {
      "max_loops", "param1", "param2", "param3", "param4"};
    names.push_back("param5");
    std::sort(names.begin(), names.end());

    std::string joined;
    for (size_t i = 0; i < names.size(); ++i) {
      joined += names[i];
      if (i + 1 < names.size()) {
        joined += ", ";
      }
    }

    RCLCPP_INFO(get_logger(), "=============Loop==============");
    RCLCPP_INFO(get_logger(), "param list: [%s]", joined.c_str());

    for (const auto & name : names) {
      const auto value = get_parameter(name);
      if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        RCLCPP_INFO(
          get_logger(),
          "parameter %s = %lld",
          name.c_str(),
          static_cast<long long>(value.as_int()));
      }
    }

    if (loop_count_ >= max_loops) {
      RCLCPP_INFO(get_logger(), "Reached max_loops=%d, shutting down", max_loops);
      timer_->cancel();
      done_ = true;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  bool done_;
  int loop_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParamDemoNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->done()) {
    executor.spin_some();
    std::this_thread::sleep_for(100ms);
  }
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
