// #include <franka_example_controllers/cartesian_velocity_example_controller.hpp>
// #include <franka_example_controllers/default_robot_behavior_utils.hpp>
// #include <franka_example_controllers/robot_utils.hpp>


// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/int32_multi_array.hpp>
// #include <Eigen/Eigen>
// #include <algorithm>

// namespace franka_example_controllers {

// class CartesianVelocityControlNode : public CartesianVelocityExampleController {
//  public:
//   CallbackReturn on_init() override {
//     return CallbackReturn::SUCCESS;
//   }

//   CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
//     franka_cartesian_velocity_ = std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
//         franka_semantic_components::FrankaCartesianVelocityInterface(k_elbow_activated_));

//     // Setup subscriber
//     command_sub_ = get_node()->create_subscription<std_msgs::msg::Int32MultiArray>(
//         "/controller_movement", 10,
//         std::bind(&CartesianVelocityControlNode::commandCallback, this, std::placeholders::_1));

//     return CallbackReturn::SUCCESS;
//   }

//  private:
//   rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr command_sub_;
//   double max_linear_velocity_ = 0.01;  // m/s
//   double max_angular_velocity_ = 0.5; // rad/s
//   const double velocity_scale = 0.05;  // Scaling factor for inputs

//   void commandCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
//     if (msg->data.size() != 6) {
//       RCLCPP_ERROR(get_node()->get_logger(), "Invalid command size: expected 6, got %zu", msg->data.size());
//       return;
//     }

//     // Read command inputs
//     int command_x = msg->data[0];
//     int command_y = msg->data[1];
//     int command_z = msg->data[2];
//     int command_roll = msg->data[3];
//     int command_pitch = msg->data[4];
//     int command_yaw = msg->data[5];

//     RCLCPP_INFO(get_node()->get_logger(), "Received command: [%d, %d, %d, %d, %d, %d]",
//                 command_x, command_y, command_z, command_roll, command_pitch, command_yaw);

//     // Apply velocity mapping and clamping
//     Eigen::Vector3d cartesian_linear_velocity(
//         std::clamp(command_x * velocity_scale, -max_linear_velocity_, max_linear_velocity_),  // X
//         std::clamp(command_y * velocity_scale, -max_linear_velocity_, max_linear_velocity_),  // Y
//         std::clamp(command_z * velocity_scale, -max_linear_velocity_, max_linear_velocity_)   // Z
//     );

//     Eigen::Vector3d cartesian_angular_velocity(
//         std::clamp(command_roll * velocity_scale, -max_angular_velocity_, max_angular_velocity_),  // Roll
//         std::clamp(command_pitch * velocity_scale, -max_angular_velocity_, max_angular_velocity_), // Pitch
//         std::clamp(command_yaw * velocity_scale, -max_angular_velocity_, max_angular_velocity_)    // Yaw
//     );

//     RCLCPP_INFO(get_node()->get_logger(), "Sending velocity command: [%f, %f, %f] Linear, [%f, %f, %f] Angular",
//                 cartesian_linear_velocity.x(), cartesian_linear_velocity.y(), cartesian_linear_velocity.z(),
//                 cartesian_angular_velocity.x(), cartesian_angular_velocity.y(), cartesian_angular_velocity.z());

//     if (!franka_cartesian_velocity_->setCommand(cartesian_linear_velocity, cartesian_angular_velocity)) {
//       RCLCPP_FATAL(get_node()->get_logger(), "Failed to set command.");
//     }
//   }
// };

// }  // namespace franka_example_controllers

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityControlNode, controller_interface::ControllerInterface)
