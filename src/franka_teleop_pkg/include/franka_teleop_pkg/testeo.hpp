#ifndef FRANKA_TELEOP_PKG_CARTESIAN_IMPEDANCE_CONTROLLER
#define FRANKA_TELEOP_PKG_CARTESIAN_IMPEDANCE_CONTROLLER

#include <memory>

#include <Eigen/Eigen>
#include <Eigen/src/Core/IO.h>
#include <Eigen/src/Core/Matrix.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>


#include <franka_teleop_pkg_interfaces/msg/joint_positions.hpp>
#include <franka_teleop_pkg_interfaces/msg/joint_torques.hpp>
#include <franka_teleop_pkg_interfaces/msg/franka_robot_state.hpp>

//!
#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

#include "franka_teleop_pkg/visibility_control.h"

#include <franka/robot_state.h>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/node.hpp"
//!

namespace franka_teleop_pkg
{

  /**
   * 
   */
  class CartesianImpedanceController : public controller_interface::ControllerInterface {

  public:
    /// Number of joints in the controlled robot
    static constexpr const size_t N_JOINTS = 7;
  
    /// Type alias for a vector with values for each joint
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    /// Type alias for command input type
    using CmdMsg = franka_teleop_pkg_interfaces::msg::JointPositions;

    /// Type alias for commanded state output type
    using StateMsg = franka_teleop_pkg_interfaces::msg::JointTorques;

  protected:
    /**
     * Specify which hardware command interfaces are used.
     *
     * This method is called in the `active` or `inactive` state. This means 
     * that the configuration can be changed during the `on_configure` state 
     * but not afterwards.
     */
    [[nodiscard]] controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    /**
     * Specify which hardware state interfaces are used.
     *
     * This method is called in the `active` or `inactive` state. This means 
     * that the configuration can be changed during the `on_configure` state 
     * but not afterwards.
     */
    [[nodiscard]] controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    /**
     * Declare all parameters.
     */
    CallbackReturn on_init() override;

    /**
     * Perform tasks that must be performed once during the node life time.

     * The tasks to perform can be e.g. obtaining parameter values and
     * setting up topic publications/subscriptions that do not change.
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
        override;

    /**
     * Clear all state and return the node to a functionally
     * equivalent state as when first created.
     *
     * This should revert all changes during the configure
     * transition, e.g. destroying all managed
     * objects transition.
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
        override;

    /**
     * Do any final preparations to start executing.
     *
     * This may include acquiring resources that are only held while the node is
     * actually active, such as access to hardware and setting up real-time
     * buffers.
     *
     * Ideally, no preparation that requires significant time (such as
     * lengthy hardware initialisation) should be performed in this callback.
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
        override;

    /**
     * Do any cleanup after executing is stopped.
     *
     * This should reverse all changes during the activate transition.
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
        override;

    /**
     * Perform a single controller update step.
     *
     * This function should ONLY execute real-time capable code.
     *
     * @param time Current time.
     * @param period Time since the last controller update step.
     */
    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    rclcpp::Subscription<CmdMsg>::SharedPtr subscriber_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>> rt_command_ptr_;

    using RtStatePublisher = realtime_tools::RealtimePublisher<StateMsg>;
    rclcpp_lifecycle::LifecyclePublisher<StateMsg>::SharedPtr publisher_;
    std::unique_ptr<RtStatePublisher> realtime_publisher_;


    // Parameter values
    Vector7d k_gains_;
    Vector7d d_gains_;
    double velocity_filter_alpha_gain_;

    // Internal variables (controller state while active)
    Vector7d filtered_joint_velocities_;

//!
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state_;

  bool k_elbow_activated{true};

  franka_msgs::msg::FrankaRobotState robot_state_;
  franka_msgs::msg::FrankaRobotState init_robot_state_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  std::string arm_id_;
  int num_joints{7};

  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_j_d);

  // Classic cartesian controller
  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};

  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_equilibrium_pose_;

  void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
//!

  };
}

#endif // FRANKA_TELEOP_PKG_CARTESIAN_IMPEDANCE_CONTROLLER