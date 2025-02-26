#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>

class RobotController : public rclcpp::Node
{
public:
  RobotController() : Node("robot_controller"), move_group_initialized_(false)
  {
    // Create subscriber
    subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/controller_movement", 10,
      std::bind(&RobotController::control_callback, this, std::placeholders::_1));

    // Create timer to check robot state
    check_state_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RobotController::init_move_group, this));

    RCLCPP_INFO(this->get_logger(), "Robot controller starting...");
  }

private:

  rclcpp::TimerBase::SharedPtr check_state_timer_;

  void init_move_group()
  {
    if (move_group_initialized_) {
      check_state_timer_->cancel();
      return;
    }

    try {
      // Try to initialize MoveGroup
      move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "fr3_manipulator");

      // Verify robot state is available
      if (!move_group_ptr_->getCurrentState(10.0)) {
        RCLCPP_WARN(this->get_logger(), "Waiting for robot state...");
        return;
      }

      // Set tolerances
      move_group_ptr_->setGoalPositionTolerance(0.01);
      move_group_ptr_->setGoalOrientationTolerance(0.1);
      
      move_group_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Robot controller initialized successfully");
      
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Initialization attempt failed: %s. Retrying...", e.what());
    }
  }

  void control_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {

    if (!move_group_initialized_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), 
                          *this->get_clock(), 
                          5000, // Throttle to every 5 seconds
                          "MoveGroup not initialized yet. Ignoring command.");
      return;
    }

    try {
      if (!move_group_ptr_->getCurrentState()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state");
        return;
      }

      if (msg->data.size() != 6) {
        RCLCPP_ERROR(this->get_logger(), "Invalid input array size. Expected 6 elements.");
        return;
      }

    // Get current pose
    
    //moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    
    geometry_msgs::msg::Pose current_pose = move_group_ptr_->getCurrentPose().pose;
    std::vector<double> current_rpy = move_group_ptr_->getCurrentRPY();

    // Define increment values (in meters and radians)
    const double pos_increment = 0.05;  // 5cm
    const double rot_increment = 0.05;  // ~2.8 degrees

    // Clear previous targets
    move_group_ptr_->clearPoseTargets();

    // Update position based on input array
    bool pos_success = move_group_ptr_->setPositionTarget(
        current_pose.position.x + (msg->data[0] * pos_increment),
        current_pose.position.y + (msg->data[1] * pos_increment),
        current_pose.position.z + (msg->data[2] * pos_increment)
    );

    // Update orientation based on input array
    bool rpy_success = move_group_ptr_->setRPYTarget(
        current_rpy[0] + (msg->data[3] * rot_increment),
        current_rpy[1] + (msg->data[4] * rot_increment),
        current_rpy[2] + (msg->data[5] * rot_increment)
    );

    if (!pos_success || !rpy_success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set target pose!");
      return;
    }

    // Plan and execute
    move_group_ptr_->setPlanningTime(5.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_ptr_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Planning succeeded! Executing...");
      move_group_ptr_->execute(my_plan);

      // Log new position
      geometry_msgs::msg::Pose new_pose = move_group_ptr_->getCurrentPose().pose;
      std::vector<double> new_rpy = move_group_ptr_->getCurrentRPY();
      
      RCLCPP_INFO(this->get_logger(), "New Position: x=%f, y=%f, z=%f",
                  new_pose.position.x, new_pose.position.y, new_pose.position.z);
      RCLCPP_INFO(this->get_logger(), "New RPY: r=%f, p=%f, y=%f",
                  new_rpy[0], new_rpy[1], new_rpy[2]);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }

  }
   catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in control callback: %s", e.what());
    }
  }

  
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr_;
  std::thread init_thread_;
  std::atomic<bool> move_group_initialized_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}