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

#include <gmock/gmock.h>
#include <exception>
#include <rclcpp/rclcpp.hpp>

#include <franka_hardware/franka_hardware_interface.hpp>
#include <franka_hardware/model.hpp>
#include <franka_hardware/robot.hpp>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

const std::string k_position_controller{"position"};
const std::string k_velocity_controller{"velocity"};
const std::string k_effort_controller{"effort"};
const std::string k_joint_name{"joint"};
const size_t k_number_of_joints{7};
const double k_EPS{1e-5};

class MockModel : public franka_hardware::Model {};

class MockRobot : public franka_hardware::Robot {
 public:
  MOCK_METHOD(void, initializeJointPositionInterface, (), (override));
  MOCK_METHOD(void, initializeCartesianVelocityInterface, (), (override));
  MOCK_METHOD(void, initializeCartesianPoseInterface, (), (override));
  MOCK_METHOD(void, initializeTorqueInterface, (), (override));
  MOCK_METHOD(void, initializeJointVelocityInterface, (), (override));
  MOCK_METHOD(void, stopRobot, (), (override));
  MOCK_METHOD(franka::RobotState, readOnce, (), (override));
  MOCK_METHOD(MockModel*, getModel, (), (override));
  MOCK_METHOD(void, writeOnce, ((const std::array<double, 7>&)efforts), (override));
  MOCK_METHOD(void, writeOnce, ((const std::array<double, 6>&)cartesian_velocity), (override));
  MOCK_METHOD(void,
              writeOnce,
              ((const std::array<double, 6>&)cartesian_velocity,
               (const std::array<double, 2>&)elbow_command),
              (override));
  MOCK_METHOD(void, writeOnce, ((const std::array<double, 16>&)cartesian_pose), (override));
  MOCK_METHOD(void,
              writeOnce,
              ((const std::array<double, 16>&)cartesian_pose,
               (const std::array<double, 2>&)elbow_command),
              (override));
  MOCK_METHOD(void,
              setJointStiffness,
              (const franka_msgs::srv::SetJointStiffness::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setCartesianStiffness,
              (const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void, setLoad, (const franka_msgs::srv::SetLoad::Request::SharedPtr&), (override));
  MOCK_METHOD(void,
              setTCPFrame,
              (const franka_msgs::srv::SetTCPFrame::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setStiffnessFrame,
              (const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setForceTorqueCollisionBehavior,
              (const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setFullCollisionBehavior,
              (const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void, automaticErrorRecovery, (), (override));
};

inline auto createHardwareInfo() -> hardware_interface::HardwareInfo {
  hardware_interface::HardwareInfo info;
  std::unordered_map<std::string, std::string> hw_params;
  hw_params["robot_ip"] = "dummy_ip";

  info.hardware_parameters = hw_params;
  hardware_interface::InterfaceInfo command_effort_interface, command_velocity_interface,
      command_position_interface, effort_state_interface, position_state_interface,
      velocity_state_interface;

  effort_state_interface.name = hardware_interface::HW_IF_EFFORT;
  position_state_interface.name = hardware_interface::HW_IF_POSITION;
  velocity_state_interface.name = hardware_interface::HW_IF_VELOCITY;

  std::vector<hardware_interface::InterfaceInfo> state_interfaces = {
      position_state_interface, velocity_state_interface, effort_state_interface};

  command_effort_interface.name = k_effort_controller;
  command_velocity_interface.name = k_velocity_controller;
  command_position_interface.name = k_position_controller;

  for (auto i = 0U; i < k_number_of_joints; i++) {
    hardware_interface::ComponentInfo joint;

    joint.name = k_joint_name + std::to_string(i + 1);

    joint.command_interfaces.push_back(command_effort_interface);
    joint.command_interfaces.push_back(command_velocity_interface);
    joint.command_interfaces.push_back(command_position_interface);

    joint.state_interfaces = state_interfaces;

    info.joints.push_back(joint);
  }

  return info;
}
