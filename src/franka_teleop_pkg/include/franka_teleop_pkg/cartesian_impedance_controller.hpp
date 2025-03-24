#ifndef FRANKA_TELEOP_PKG_CARTESIAN_IMPEDANCE_CONTROLLER_HPP
#define FRANKA_TELEOP_PKG_CARTESIAN_IMPEDANCE_CONTROLLER_HPP

#include <memory>
#include <mutex>

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <franka_teleop_pkg_interfaces/msg/joint_positions.hpp>
#include <franka_teleop_pkg_interfaces/msg/joint_torques.hpp>
#include <franka_teleop_pkg_interfaces/msg/franka_robot_state.hpp>

#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"
#include "franka_teleop_pkg/visibility_control.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace franka_semantic_components {
  class FrankaRobotState;
  class FrankaRobotModel;
}

namespace franka_teleop_pkg
{

/**
 * @brief Cartesian impedance controller for the Franka Research 3 robot.
 *
 * This controller implements Cartesian impedance control, where a desired Cartesian pose
 * is the input. It uses inverse kinematics to convert this pose into joint space, followed
 * by a joint impedance control strategy to move the robot.
 */
class CartesianImpedanceController : public controller_interface::ControllerInterface {

public:
  /// Number of joints in the controlled robot.
  static constexpr size_t N_JOINTS = 7;

  /// Type alias for a vector with values for each joint.
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  /// Type alias for command input type.
  using CmdMsg = franka_teleop_pkg_interfaces::msg::JointPositions;

  /// Type alias for commanded state output type.
  using StateMsg = franka_teleop_pkg_interfaces::msg::JointTorques;

  /**
   * @brief Saturate torque rate to ensure smooth control.
   * 
   * Limits the change in torque command between successive updates.
   * 
   * @param tau_d_calculated Calculated desired torque vector.
   * @param tau_j_d Current joint torque vector.
   * @return Saturated torque vector.
   */
  Vector7d saturateTorqueRate(
      const Vector7d& tau_d_calculated,
      const Vector7d& tau_j_d);

protected:
  /**
   * @brief Specify which hardware command interfaces are used.
   * 
   * This method configures the individual effort interfaces for each joint.
   * 
   * @return InterfaceConfiguration for command interfaces.
   */
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  /**
   * @brief Specify which hardware state interfaces are used.
   * 
   * Configures position and velocity interfaces for each joint and appends additional
   * state interfaces from the robot model and state.
   * 
   * @return InterfaceConfiguration for state interfaces.
   */
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  /**
   * @brief Declare and initialize controller parameters.
   *
   * Declares parameters for gains, compliance, topics, etc. and initializes
   * the default target pose and Cartesian compliance matrices.
   *
   * @return CallbackReturn::SUCCESS on success, ERROR otherwise.
   */
  CallbackReturn on_init() override;

  /**
   * @brief Configure the controller.
   *
   * Obtains parameters, initializes the robot state/model components, and sets up
   * subscriptions and publishers.
   *
   * @param previous_state The previous lifecycle state.
   * @return CallbackReturn::SUCCESS on success, ERROR otherwise.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Clean up the controller.
   *
   * Resets internal variables and releases interfaces and communication handles.
   *
   * @param previous_state The previous lifecycle state.
   * @return CallbackReturn::SUCCESS on success, ERROR otherwise.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Activate the controller.
   *
   * Initializes internal state (capturing current robot state) and sets the initial
   * target pose and nullspace configuration.
   *
   * @param previous_state The previous lifecycle state.
   * @return CallbackReturn::SUCCESS on success, ERROR otherwise.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Deactivate the controller.
   *
   * Cleans up after execution is stopped, releasing hardware interfaces.
   *
   * @param previous_state The previous lifecycle state.
   * @return CallbackReturn::SUCCESS on success, ERROR otherwise.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Perform a single controller update step.
   *
   * Reads the current robot state, computes the Cartesian error, maps it into joint space,
   * applies PD control (with nullspace stabilization), and writes torque commands.
   *
   * @param time Current time.
   * @param period Time since the last update.
   * @return controller_interface::return_type::OK on success, ERROR otherwise.
   */
  controller_interface::return_type update(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

  /**
   * @brief Callback for equilibrium pose updates.
   *
   * Processes incoming PoseStamped messages to update the desired target position and orientation.
   *
   * @param msg Shared pointer to the new PoseStamped message.
   */
  void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

private:
  // --- Private Helper Functions ---
  /**
   * @brief Initialize the Cartesian compliance matrices.
   *
   * Computes and sets the Cartesian stiffness and damping matrices using the
   * parameterized translational and rotational stiffness values.
   */
  void initializeComplianceMatrices();

  // --- Private Member Variables ---
  // Subscriptions and publishers.
  rclcpp::Subscription<CmdMsg>::SharedPtr subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>> rt_command_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_equilibrium_pose_;
  using RtStatePublisher = realtime_tools::RealtimePublisher<StateMsg>;
  rclcpp_lifecycle::LifecyclePublisher<StateMsg>::SharedPtr publisher_;
  std::unique_ptr<RtStatePublisher> realtime_publisher_;

  // Franka robot semantic components.
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

  // Robot state messages.
  franka_msgs::msg::FrankaRobotState robot_state_;
  franka_msgs::msg::FrankaRobotState init_robot_state_;

  // Arm identifier.
  std::string arm_id_;

  // Constants and identifiers.
  static constexpr int num_joints = 7;
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
  const double delta_tau_max_{1.0};

  // Parameter values.
  Vector7d k_gains_;
  Vector7d d_gains_;
  double velocity_filter_alpha_gain_{0.99};

  // Cartesian control parameters.
  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};

  // New compliance parameters (parameterized).
  double translational_stiffness_;  ///< Parameterized translational stiffness.
  double rotational_stiffness_;     ///< Parameterized rotational stiffness.

  // Cartesian stiffness and damping matrices.
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;

  // Nullspace configuration.
  Vector7d q_d_nullspace_;

  // Target pose variables.
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  // Internal variables for controller state while active.
  Vector7d filtered_joint_velocities_;
  bool k_elbow_activated{true};
};

} // namespace franka_teleop_pkg

#endif // FRANKA_TELEOP_PKG_CARTESIAN_IMPEDANCE_CONTROLLER_HPP
