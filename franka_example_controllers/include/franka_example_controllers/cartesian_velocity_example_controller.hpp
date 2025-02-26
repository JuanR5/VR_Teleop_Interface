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

#pragma once

#include <string>

#include <Eigen/Eigen>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp> 

#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The cartesian velocity example controller
 */
class CartesianVelocityExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface>
      franka_cartesian_velocity_;

  const double k_time_max_{4.0};
  const double k_v_max_{0.05};
  const double k_angle_{M_PI / 4.0};
  const bool k_elbow_activated_{false};

  //!

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr movement_subscriber_;

  Eigen::Vector3d target_linear_velocity_;
  Eigen::Vector3d desired_linear_velocity_;
  Eigen::Vector3d target_angular_velocity_;
  Eigen::Vector3d desired_angular_velocity_;
  //!

  rclcpp::Duration elapsed_time_ = rclcpp::Duration(0, 0);
};

}  // namespace franka_example_controllers
