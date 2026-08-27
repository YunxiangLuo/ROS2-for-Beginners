#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "action_demo_interfaces/action/do_dishes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class DoDishesServer : public rclcpp::Node
{
public:
  using DoDishes = action_demo_interfaces::action::DoDishes;
  using GoalHandleDoDishes = rclcpp_action::ServerGoalHandle<DoDishes>;

  DoDishesServer()
  : Node("dishes_server")
  {
    action_server_ = rclcpp_action::create_server<DoDishes>(
      this,
      "dishes",
      std::bind(&DoDishesServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DoDishesServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&DoDishesServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const DoDishes::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal for dishwasher_id=%u", goal->dishwasher_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDoDishes> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDoDishes> goal_handle)
  {
    std::thread(std::bind(&DoDishesServer::execute, this, std::placeholders::_1), goal_handle)
      .detach();
  }

  void execute(const std::shared_ptr<GoalHandleDoDishes> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DoDishes::Feedback>();
    auto result = std::make_shared<DoDishes::Result>();

    for (int step = 1; step <= 5; ++step) {
      if (goal_handle->is_canceling()) {
        result->total_dishes_cleaned = static_cast<uint32_t>((step - 1) * goal->dishwasher_id);
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return;
      }

      feedback->percent_complete = static_cast<float>(step) * 20.0F;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "Feedback %.1f%%", feedback->percent_complete);
      std::this_thread::sleep_for(500ms);
    }

    result->total_dishes_cleaned = goal->dishwasher_id * 5;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded, cleaned %u dishes", result->total_dishes_cleaned);
  }

  rclcpp_action::Server<DoDishes>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DoDishesServer>());
  rclcpp::shutdown();
  return 0;
}
