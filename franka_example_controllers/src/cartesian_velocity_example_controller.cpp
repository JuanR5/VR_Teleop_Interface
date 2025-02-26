// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_example_controllers/cartesian_velocity_example_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_velocity_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type CartesianVelocityExampleController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;

  //!
  // Apply dampers to smooth out velocity changes
  double damping_factor = 0.05;  // Adjust this value to control the damping effect
  target_linear_velocity_ = target_linear_velocity_ + damping_factor * (desired_linear_velocity_ - target_linear_velocity_);
  target_angular_velocity_ = target_angular_velocity_ + damping_factor * (desired_angular_velocity_ - target_angular_velocity_);

  // Clamp velocities to maximum limits
  double max_linear_velocity = 0.015;  // Maximum linear velocity
  double max_angular_velocity = 0.03;  // Maximum angular velocity

  target_linear_velocity_ = target_linear_velocity_.cwiseMax(-max_linear_velocity).cwiseMin(max_linear_velocity);
  target_angular_velocity_ = target_angular_velocity_.cwiseMax(-max_angular_velocity).cwiseMin(max_angular_velocity);


  Eigen::Vector3d cartesian_linear_velocity = target_linear_velocity_;
  Eigen::Vector3d cartesian_angular_velocity = target_angular_velocity_;

// !
  // double cycle = std::floor(pow(
  //     -1.0,
  //     (elapsed_time_.seconds() - std::fmod(elapsed_time_.seconds(), k_time_max_)) / k_time_max_));
  // double v =
  //     cycle * k_v_max_ / 2.0 * (1.0 - std::cos(2.0 * M_PI / k_time_max_ * elapsed_time_.seconds()));
  // double v_x = std::cos(k_angle_) * v;
  // double v_z = -std::sin(k_angle_) * v;
  // double v_y = std::sin(k_angle_) * v;

  // Eigen::Vector3d cartesian_linear_velocity(v_x, v_y, v_z);
  // Eigen::Vector3d cartesian_angular_velocity(0.0, 0.0, 0.0);
// !

  if (franka_cartesian_velocity_->setCommand(cartesian_linear_velocity,
                                             cartesian_angular_velocity)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

CallbackReturn CartesianVelocityExampleController::on_init() {
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
          franka_semantic_components::FrankaCartesianVelocityInterface(k_elbow_activated_));

  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(robot_utils::time_out);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

// !

  // Low-pass filter factor (tune between 0.0 - 1.0)
  const double alpha = 0.2;  // Higher value reacts faster, lower value smooths more


  // Create a subscriber to the /controller_movement topic
  movement_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "/controller_movement", 10,
    [this, alpha](const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Extract linear velocities
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double vz = msg->linear.z;

        // Extract angular velocities
        double wx = msg->angular.x;
        double wy = msg->angular.y;
        double wz = msg->angular.z;

        // Validate message (check for NaN or infinite values)
        if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz) ||
            !std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz)) {
            RCLCPP_WARN(get_node()->get_logger(), "Received invalid velocity value, ignoring.");
            return;
        }

        // Apply low-pass filtering
        desired_linear_velocity_ = alpha * Eigen::Vector3d(vx, vy, vz) +
                                  (1 - alpha) * desired_linear_velocity_;

        desired_angular_velocity_ = alpha * Eigen::Vector3d(wx, wy, wz) +
                                   (1 - alpha) * desired_angular_velocity_;

        // Clamp desired velocities to maximum limits
        double max_linear_velocity = 0.015;  // Maximum linear velocity
        double max_angular_velocity = 0.03;  // Maximum angular velocity

        desired_linear_velocity_ = desired_linear_velocity_.cwiseMax(-max_linear_velocity).cwiseMin(max_linear_velocity);
        desired_angular_velocity_ = desired_angular_velocity_.cwiseMax(-max_angular_velocity).cwiseMin(max_angular_velocity);
  
    });
// !
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerInterface)
