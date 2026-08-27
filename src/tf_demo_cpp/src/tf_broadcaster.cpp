#include <chrono>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class TfBroadcasterNode : public rclcpp::Node
{
public:
  TfBroadcasterNode()
  : Node("tf_broadcaster"), yaw_(1.57)
  {
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(1s, std::bind(&TfBroadcasterNode::broadcast, this));
  }

private:
  void broadcast()
  {
    yaw_ += 0.1;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "link1";
    t.transform.translation.x = 1.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 3.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    broadcaster_->sendTransform(t);
    RCLCPP_INFO(get_logger(), "Broadcasted transform yaw=%.2f", yaw_);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double yaw_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}
