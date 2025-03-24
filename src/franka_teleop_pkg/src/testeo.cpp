#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <franka_msgs/msg/franka_robot_state.hpp>
#include <chrono>
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;

/**
 * @brief Node that publishes oscillatory equilibrium poses.
 *
 * This node subscribes to the robot state topic ("franka_robot_state") to obtain
 * the initial end-effector pose. Then it publishes PoseStamped messages on the
 * "equilibrium_pose" topic where the x and z positions oscillate with a given
 * amplitude and frequency.
 */
class OscillatoryPosePublisher : public rclcpp::Node {
public:
  OscillatoryPosePublisher()
    : Node("oscillatory_pose_publisher"),
      amplitude_(0.1),      // 10 m amplitude (adjust as needed)
      frequency_(0.5),       // 0.5 Hz oscillation frequency
      initial_pose_received_(false)
  {
    // Subscribe to robot state to get the current end-effector pose.
    // (Assumes the robot state is published on "franka_robot_state")
    robot_state_sub_ = this->create_subscription<franka_msgs::msg::FrankaRobotState>(
      "franka_robot_state", 10,
      std::bind(&OscillatoryPosePublisher::robotStateCallback, this, std::placeholders::_1)
    );
    // Publisher for equilibrium pose.
    equilibrium_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("equilibrium_pose", 10);
    // Create a timer to send new pose commands.
    timer_ = this->create_wall_timer(10ms, std::bind(&OscillatoryPosePublisher::timerCallback, this));
  }

private:
  /**
   * @brief Callback for receiving the robot state.
   *
   * Extracts the current end-effector pose from the robot state message and
   * stores it as the initial pose (if not already set).
   *
   * @param msg Shared pointer to the incoming robot state message.
   */
  void robotStateCallback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // If we haven't captured an initial pose yet, do so now.
    if (!initial_pose_received_) {
      // Assuming the end-effector pose is in msg->o_t_ee.pose.
      initial_pose_ = msg->o_t_ee.pose;
      initial_pose_received_ = true;
      start_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Initial end-effector pose received.");
    }
  }

  /**
   * @brief Timer callback that publishes oscillatory equilibrium poses.
   *
   * Computes oscillatory offsets (based on sine/cosine functions) relative to the initial pose
   * and publishes a new equilibrium pose.
   */
  void timerCallback()
  {
    if (!initial_pose_received_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for initial robot state...");
      return;
    }
    rclcpp::Time current_time = this->now();
    double t = (current_time - start_time_).seconds();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = current_time;
    pose_msg.header.frame_id = "base_link";

    // Get the initial pose safely.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pose_msg.pose = initial_pose_;
    }
    // Add oscillatory offsets along x and z axes.
    pose_msg.pose.position.x += amplitude_ * std::sin(2.0 * M_PI * frequency_ * t);
    pose_msg.pose.position.z += amplitude_ * std::cos(2.0 * M_PI * frequency_ * t);

    equilibrium_pub_->publish(pose_msg);
  }

  // Node publisher and subscriber.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr equilibrium_pub_;
  rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr robot_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Stored initial end-effector pose.
  geometry_msgs::msg::Pose initial_pose_;
  bool initial_pose_received_;
  rclcpp::Time start_time_;

  // Oscillation parameters.
  double amplitude_;  // Amplitude of oscillation in meters.
  double frequency_;  // Frequency in Hz.

  // Mutex to protect access to the initial pose and start time.
  std::mutex mutex_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OscillatoryPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
