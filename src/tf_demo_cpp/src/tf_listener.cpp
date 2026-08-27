#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TfListenerNode : public rclcpp::Node
{
public:
  TfListenerNode()
  : Node("tf_listener"),
    buffer_(get_clock()),
    listener_(buffer_)
  {
    timer_ = create_wall_timer(1s, std::bind(&TfListenerNode::lookup, this));
  }

private:
  void lookup()
  {
    try {
      const auto transform = buffer_.lookupTransform("base_link", "link1", tf2::TimePointZero);
      const auto & tr = transform.transform.translation;
      const auto & rot = transform.transform.rotation;
      RCLCPP_INFO(
        get_logger(),
        "translation x=%.2f y=%.2f z=%.2f | quaternion w=%.4f x=%.4f y=%.4f z=%.4f",
        tr.x, tr.y, tr.z, rot.w, rot.x, rot.y, rot.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Transform lookup failed: %s", ex.what());
    }
  }

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfListenerNode>());
  rclcpp::shutdown();
  return 0;
}
