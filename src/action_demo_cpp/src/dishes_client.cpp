#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

#include "action_demo_interfaces/action/do_dishes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class DoDishesClient : public rclcpp::Node
{
public:
  using DoDishes = action_demo_interfaces::action::DoDishes;
  using GoalHandleDoDishes = rclcpp_action::ClientGoalHandle<DoDishes>;

  DoDishesClient()
  : Node("dishes_client")
  {
    client_ = rclcpp_action::create_client<DoDishes>(this, "dishes");
  }

  int run()
  {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Action server not available");
      return 1;
    }

    DoDishes::Goal goal_msg;
    goal_msg.dishwasher_id = 2;

    auto send_goal_options = rclcpp_action::Client<DoDishes>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [this](GoalHandleDoDishes::SharedPtr, const std::shared_ptr<const DoDishes::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), "Feedback %.1f%%", feedback->percent_complete);
      };

    auto goal_handle_future = client_->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal");
      return 1;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
      return 1;
    }

    auto result_future = client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to get result");
      return 1;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "Action did not succeed");
      return 1;
    }

    RCLCPP_INFO(
      get_logger(),
      "Yay! The dishes are now clean. Total cleaned: %u",
      wrapped_result.result->total_dishes_cleaned);
    return 0;
  }

private:
  rclcpp_action::Client<DoDishes>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DoDishesClient>();
  const int rc = node->run();
  rclcpp::shutdown();
  return rc;
}
