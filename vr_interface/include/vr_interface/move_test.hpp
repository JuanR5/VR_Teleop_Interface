#ifndef VR_INTERFACE_MOVE_TEST
#define VR_INTERFACE_MOVE_TEST

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace vr_interface {

class MoveTest : public rclcpp::Node {
public:
  MoveTest();

private:
  void timer_callback();
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vr_interface

#endif  // VR_INTERFACE_MOVE_TEST
