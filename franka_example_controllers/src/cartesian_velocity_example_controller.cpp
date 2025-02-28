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
#include <rclcpp_action/rclcpp_action.hpp>
#include <franka_msgs/action/error_recovery.hpp>

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

  try {
    // Apply low-pass filtering to smooth velocity commands
    double alpha = 0.5;  // Higher value smooths more, lower value reacts faster
    target_linear_velocity_ = (1 - alpha) * desired_linear_velocity_ + alpha * target_linear_velocity_;
    target_angular_velocity_ = (1 - alpha) * desired_angular_velocity_ + alpha * target_angular_velocity_;

    // Clamp velocities to maximum limits
    double max_linear_velocity = 0.01;  // Maximum linear velocity
    double max_angular_velocity = 0.01;  // Maximum angular velocity
    target_linear_velocity_ = target_linear_velocity_.cwiseMax(-max_linear_velocity).cwiseMin(max_linear_velocity);
    target_angular_velocity_ = target_angular_velocity_.cwiseMax(-max_angular_velocity).cwiseMin(max_angular_velocity);

    // Limit acceleration
    double max_linear_acceleration = 0.005;  // Maximum linear acceleration
    double max_angular_acceleration = 0.005;  // Maximum angular acceleration
    Eigen::Vector3d linear_acceleration = (target_linear_velocity_ - previous_linear_velocity_) / period.seconds();
    Eigen::Vector3d angular_acceleration = (target_angular_velocity_ - previous_angular_velocity_) / period.seconds();

    linear_acceleration = linear_acceleration.cwiseMax(-max_linear_acceleration).cwiseMin(max_linear_acceleration);
    angular_acceleration = angular_acceleration.cwiseMax(-max_angular_acceleration).cwiseMin(max_angular_acceleration);

    target_linear_velocity_ = target_linear_velocity_ + linear_acceleration * period.seconds();
    target_angular_velocity_ = target_angular_velocity_ + angular_acceleration * period.seconds();

    Eigen::Vector3d cartesian_linear_velocity = target_linear_velocity_;
    Eigen::Vector3d cartesian_angular_velocity = target_angular_velocity_;

    // Log velocities
    RCLCPP_INFO(get_node()->get_logger(), "Target Linear Velocity: [%f, %f, %f]",
                target_linear_velocity_.x(), target_linear_velocity_.y(), target_linear_velocity_.z());
    RCLCPP_INFO(get_node()->get_logger(), "Target Angular Velocity: [%f, %f, %f]",
                target_angular_velocity_.x(), target_angular_velocity_.y(), target_angular_velocity_.z());

    // Send command to the robot
    if (franka_cartesian_velocity_->setCommand(cartesian_linear_velocity, cartesian_angular_velocity)) {
      previous_linear_velocity_ = target_linear_velocity_;
      previous_angular_velocity_ = target_angular_velocity_;
      return controller_interface::return_type::OK;
    } else {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Set command failed. Did you activate the elbow command interface?");
      recoverFromError();  // Trigger error recovery
      return controller_interface::return_type::ERROR;
    }

  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_node()->get_logger(), "Caught exception in CartesianVelocityExampleController: %s", e.what());
    recoverFromError();  // Trigger error recovery
    return controller_interface::return_type::ERROR;
  }
  } catch (const franka::ControlException& e) {
    RCLCPP_FATAL(get_node()->get_logger(), "Franka control exception: %s", e.what());
    recoverFromError();
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
      "service_server/set_full_collision_behavior"); //! check later
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

  // Initialize error recovery client
  error_recovery_client_ = rclcpp_action::create_client<franka_msgs::action::ErrorRecovery>(
      get_node(), "/action_server/error_recovery");

  // Subscribe to movement commands
  const double deadband = 0.001;
  movement_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "/controller_movement", 10,
    [this, deadband](const geometry_msgs::msg::Twist::SharedPtr msg) {
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        double vz = msg->linear.z;

        double wx = msg->angular.x;
        double wy = msg->angular.y;
        double wz = msg->angular.z;

        // Validate message
        if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz) ||
            !std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz)) {
            RCLCPP_WARN(get_node()->get_logger(), "Received invalid velocity value, ignoring.");
            return;
        }

        // Apply deadband
        if (std::abs(vx) < deadband) vx = 0.0;
        if (std::abs(vy) < deadband) vy = 0.0;
        if (std::abs(vz) < deadband) vz = 0.0;
        if (std::abs(wx) < deadband) wx = 0.0;
        if (std::abs(wy) < deadband) wy = 0.0;
        if (std::abs(wz) < deadband) wz = 0.0;

        double scale_linear = 0.005;  // Scaling factor for linear velocity
        double scale_angular = 0.005; // Scaling factor for angular velocity

        // Scale input values
        desired_linear_velocity_ = Eigen::Vector3d(vx * scale_linear, vy * scale_linear, vz * scale_linear);
        desired_angular_velocity_ = Eigen::Vector3d(wx * scale_angular, wy * scale_angular, wz * scale_angular);

        // Clamp desired velocities
        double max_linear_velocity = 0.01;
        double max_angular_velocity = 0.01;

        desired_linear_velocity_ = desired_linear_velocity_.cwiseMax(-max_linear_velocity).cwiseMin(max_linear_velocity);
        desired_angular_velocity_ = desired_angular_velocity_.cwiseMax(-max_angular_velocity).cwiseMin(max_angular_velocity);

        // Log input velocities
        RCLCPP_INFO(get_node()->get_logger(), "Input Linear Velocity: [%f, %f, %f]", vx, vy, vz);
        RCLCPP_INFO(get_node()->get_logger(), "Input Angular Velocity: [%f, %f, %f]", wx, wy, wz);
    });

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

void CartesianVelocityExampleController::recoverFromError() {
  if (!error_recovery_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error recovery action server not available.");
    return;
  }

  auto goal_msg = franka_msgs::action::ErrorRecovery::Goal();
  auto send_goal_options = rclcpp_action::Client<franka_msgs::action::ErrorRecovery>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::ErrorRecovery>::SharedPtr& goal_handle) {
          if (!goal_handle) {
              RCLCPP_ERROR(get_node()->get_logger(), "Error recovery goal was rejected by server.");
          } else {
              RCLCPP_INFO(get_node()->get_logger(), "Error recovery goal accepted by server.");
          }
      };

  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::ErrorRecovery>::WrappedResult& result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_INFO(get_node()->get_logger(), "Error recovery completed successfully.");
          } else {
              RCLCPP_ERROR(get_node()->get_logger(), "Error recovery failed.");
          }
      };

  error_recovery_client_->async_send_goal(goal_msg, send_goal_options);
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerInterface)