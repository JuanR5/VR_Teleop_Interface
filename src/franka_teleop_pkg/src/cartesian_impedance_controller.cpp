#include <franka_teleop_pkg/cartesian_impedance_controller.hpp>
#include <franka_teleop_pkg/pseudo_inversion.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <controller_interface/controller_interface_base.hpp>
#include <franka/robot_state.h>

namespace franka_teleop_pkg {


// Define number of joints as a constant.
static constexpr size_t N_JOINTS = 7;

/**
 * @brief Normalize a quaternion.
 * @param q Input quaternion.
 * @return Normalized quaternion.
 */
static inline Eigen::Quaterniond normalizeQuaternion(const Eigen::Quaterniond &q) {
  return q.normalized();
}

/**
 * @brief Compute the error quaternion.
 *
 * Computes the rotation error as current.inverse() * desired.
 *
 * @param current The current orientation.
 * @param desired The desired orientation.
 * @return Error quaternion.
 */
static inline Eigen::Quaterniond computeQuaternionError(const Eigen::Quaterniond &current,
                                                          const Eigen::Quaterniond &desired) {
  return current.inverse() * desired;
}

/**
 * @brief Update orientation target ensuring hemisphere consistency.
 *
 * Normalizes the new target and flips its sign if necessary so that its dot product
 * with the current target is non-negative.
 *
 * @param current_target The current orientation target.
 * @param new_target The new desired orientation.
 * @return Updated orientation target.
 */
static inline Eigen::Quaterniond updateOrientationTarget(const Eigen::Quaterniond &current_target,
                                                           const Eigen::Quaterniond &new_target) {
  Eigen::Quaterniond updated = new_target.normalized();
  if (current_target.coeffs().dot(updated.coeffs()) < 0.0) {
    updated.coeffs() = -updated.coeffs();
  }
  return updated;
}

controller_interface::InterfaceConfiguration 
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::string arm_prefix = (arm_id_.empty() ? "fr3" : arm_id_) + std::string("_joint");
  for (size_t index = 1; index <= N_JOINTS; index++) {
    config.names.push_back(arm_prefix + std::to_string(index) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration 
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::string arm_prefix = (arm_id_.empty() ? "fr3" : arm_id_) + std::string("_joint");
  for (size_t index = 1; index <= N_JOINTS; index++) {
    config.names.push_back(arm_prefix + std::to_string(index) + "/position");
    config.names.push_back(arm_prefix + std::to_string(index) + "/velocity");
  }
  if (franka_robot_model_) {
    for (const auto& name : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(name);
    }
  }
  if (franka_robot_state_) {
    for (const auto& name : franka_robot_state_->get_state_interface_names()) {
      config.names.push_back(name);
    }
  }
  return config;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_init() {
  try {
    // Declare only the required parameters.
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<double>("translational_stiffness", 600.0);
    auto_declare<double>("rotational_stiffness", 100.0);
    auto_declare<std::string>("arm_id", "fr3");

    // Initialize desired pose.
    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    // Retrieve compliance parameters.
    translational_stiffness_ = get_node()->get_parameter("translational_stiffness").as_double();
    rotational_stiffness_ = get_node()->get_parameter("rotational_stiffness").as_double();

    // Initialize compliance matrices.
    initializeComplianceMatrices();
    cartesian_stiffness_target_ = cartesian_stiffness_;
    cartesian_damping_target_ = cartesian_damping_;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("CartesianImpedanceController"), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::initializeComplianceMatrices() {
  // Set stiffness matrices.
  cartesian_stiffness_.setZero();
  cartesian_stiffness_.topLeftCorner(3, 3) = translational_stiffness_ * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3) = rotational_stiffness_ * Eigen::Matrix3d::Identity();
  
  // Compute damping matrices (damping ratio = 1).
  cartesian_damping_.setZero();
  cartesian_damping_.topLeftCorner(3, 3) = 2.0 * sqrt(translational_stiffness_) * Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3) = 2.0 * sqrt(rotational_stiffness_) * Eigen::Matrix3d::Identity();
}

controller_interface::CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  try {
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    if (!get_node()->has_parameter("robot_description")) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing required parameter: robot_description");
      return controller_interface::CallbackReturn::ERROR;
    }
    auto robot_description = get_node()->get_parameter("robot_description").as_string();
    // Initialize semantic components.
    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
        arm_id_ + "/" + k_robot_state_interface_name, robot_description);
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        arm_id_ + "/" + k_robot_model_interface_name, arm_id_ + "/" + k_robot_state_interface_name);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during robot state/model initialization: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get and validate gains.
  auto k_gains_param = get_node()->get_parameter("k_gains").as_double_array();
  if (k_gains_param.empty() || k_gains_param.size() != N_JOINTS) {
    RCLCPP_FATAL(get_node()->get_logger(), "Invalid k_gains parameter (size %ld)", k_gains_param.size());
    return controller_interface::CallbackReturn::FAILURE;
  }
  auto d_gains_param = get_node()->get_parameter("d_gains").as_double_array();
  if (d_gains_param.empty() || d_gains_param.size() != N_JOINTS) {
    RCLCPP_FATAL(get_node()->get_logger(), "Invalid d_gains parameter (size %ld)", d_gains_param.size());
    return controller_interface::CallbackReturn::FAILURE;
  }
  for (size_t index = 0; index < N_JOINTS; index++) {
    k_gains_(static_cast<int>(index)) = k_gains_param.at(index);
    d_gains_(static_cast<int>(index)) = d_gains_param.at(index);
  }

  // Set up equilibrium pose subscription.
  try {
    sub_equilibrium_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "new_goal_pose", 20,
      std::bind(&CartesianImpedanceController::equilibriumPoseCallback, this, std::placeholders::_1)
    );
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during equilibrium pose subscription creation: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Reset gains.
  for (size_t index = 0; index < N_JOINTS; index++) {
    k_gains_(static_cast<int>(index)) = 0;
    d_gains_(static_cast<int>(index)) = 0;
  }
  // Reset subscriptions and publishers.
  sub_equilibrium_pose_.reset();
  realtime_publisher_.reset();

  RCLCPP_INFO(get_node()->get_logger(), "Cleanup successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  filtered_joint_velocities_.setZero();
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>>(nullptr);

  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  init_robot_state_ = franka_msgs::msg::FrankaRobotState();
  franka_robot_state_->get_values_as_message(init_robot_state_);

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(init_robot_state_.measured_joint_state.position.data());
  Eigen::Vector3d position_init(
      init_robot_state_.o_t_ee.pose.position.x,
      init_robot_state_.o_t_ee.pose.position.y,
      init_robot_state_.o_t_ee.pose.position.z);
  Eigen::Quaterniond orientation_init(
      init_robot_state_.o_t_ee.pose.orientation.w, 
      init_robot_state_.o_t_ee.pose.orientation.x,
      init_robot_state_.o_t_ee.pose.orientation.y, 
      init_robot_state_.o_t_ee.pose.orientation.z);
      
  // Build transform using Eigen composition.
  Eigen::Affine3d initial_transform = Eigen::Translation3d(position_init) * orientation_init;
  
  position_d_ = initial_transform.translation();
  orientation_d_ = initial_transform.rotation();
  // position_d_target_ = initial_transform.translation();
  // orientation_d_target_ = initial_transform.rotation();

  new_goal_pose_received_ = true;  // triggers interpolation filter immediately

//?  // Log the initialization pose and orientation.
  RCLCPP_INFO(get_node()->get_logger(), "Initialization Pose: [x: %f, y: %f, z: %f]",
  position_init.x(), position_init.y(), position_init.z());
  RCLCPP_INFO(get_node()->get_logger(), "Initialization Orientation: [w: %f, x: %f, y: %f, z: %f]",
  orientation_init.w(), orientation_init.x(), orientation_init.y(), orientation_init.z());
//?

  q_d_nullspace_ = q_initial;

  RCLCPP_INFO(get_node()->get_logger(), "Activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  filtered_joint_velocities_.setZero();
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>>(nullptr);

  franka_robot_state_->release_interfaces();
  franka_robot_model_->release_interfaces();

  RCLCPP_INFO(get_node()->get_logger(), "Deactivate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

// !
// !
controller_interface::return_type CartesianImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
  
  // --- 1. Acquire current robot state ---
  robot_state_ = franka_msgs::msg::FrankaRobotState();
  franka_robot_state_->get_values_as_message(robot_state_);

  // static bool is_first_cycle = true;
  // if (is_first_cycle) {
  //   for (size_t i = 0; i < N_JOINTS; i++) {
  //     command_interfaces_[i].set_value(0.0);
  //   }
  //   is_first_cycle = false;
  //   return controller_interface::return_type::OK;
  // }

  // --- 2. Retrieve robot model data (mass, coriolis, jacobian) ---
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);

  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // --- 3. Map current joint state ---
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state_.measured_joint_state.position.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state_.measured_joint_state.velocity.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_j_d(robot_state_.measured_joint_state.effort.data());

  // --- 4. Compute current end-effector pose ---
  Eigen::Vector3d position(robot_state_.o_t_ee.pose.position.x,
                           robot_state_.o_t_ee.pose.position.y,
                           robot_state_.o_t_ee.pose.position.z);
  Eigen::Quaterniond orientation(robot_state_.o_t_ee.pose.orientation.w,
                                 robot_state_.o_t_ee.pose.orientation.x,
                                 robot_state_.o_t_ee.pose.orientation.y,
                                 robot_state_.o_t_ee.pose.orientation.z);
  Eigen::Affine3d transform = Eigen::Translation3d(position) * orientation;

  // --- 6. Compute Cartesian error ---
  Eigen::Matrix<double, 6, 1> position_error;
  position_error.head(3) = position - position_d_;
  
  // Ensure proper hemisphere alignment.
  orientation = normalizeQuaternion(orientation);
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() = -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion = computeQuaternionError(orientation, orientation_d_);
  position_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  position_error.tail(3) = -transform.rotation() * position_error.tail(3);

  //?
  RCLCPP_INFO(get_node()->get_logger(),
            "Position error: [%.4f, %.4f, %.4f] | Orientation error: [%.4f, %.4f, %.4f]",
            position_error(0), position_error(1), position_error(2),
            position_error(3), position_error(4), position_error(5));
  //?

  // --- Add Deadband to Prevent Acting on Tiny Errors ---
  const double position_deadband_threshold = 5e-3;  // in meters (adjust as needed)
  double pos_error_norm = position_error.head(3).norm();
  if (pos_error_norm < position_deadband_threshold) {
    position_error.head(3).setZero();
  }

  const double orientation_deadband_threshold = 1e-2;  // in radians (adjust as needed)
  double orient_error_norm = position_error.tail(3).norm();
  if (orient_error_norm < orientation_deadband_threshold) {
    position_error.tail(3).setZero();
  }

  // --- 7. Compute control torques ---
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);
  Eigen::MatrixXd jacobian_transpose_pinv = pseudoInverse(jacobian.transpose(), true);
  
  // Eigen::VectorXd cartesian_velocity = jacobian * dq;
  // for (int i = 0; i < cartesian_velocity.size(); ++i) {
  //   if (std::abs(cartesian_velocity[i]) < 1e-3) {
  //     cartesian_velocity[i] = 0.0;
  //   }
  // }
  tau_task = jacobian.transpose() * (-cartesian_stiffness_ * position_error - cartesian_damping_ * jacobian * dq);
  tau_nullspace = (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
                  (nullspace_stiffness_ * (q_d_nullspace_ - q) - (2.0 * sqrt(nullspace_stiffness_)) * dq);
  tau_d = tau_task + tau_nullspace + coriolis;
  tau_d = saturateTorqueRate(tau_d, tau_j_d);

  // --- Apply optional torque ramp-up to prevent startup spikes ---
  static size_t init_counter = 0;
  static constexpr size_t ramp_duration = 10000; // ~0.5s at 1000Hz
  double alpha = std::min(1.0, static_cast<double>(init_counter++) / ramp_duration);
  tau_d *= alpha;

  // --- 8. Send computed torques ---
  std::ostringstream torque_stream;
  for (size_t i = 0; i < N_JOINTS; i++) {
    command_interfaces_[i].set_value(tau_d[i]);
    torque_stream << "Joint " << i << ": " << tau_d[i] << "  ";
  }
  RCLCPP_INFO(get_node()->get_logger(), "Commanded torques: %s", torque_stream.str().c_str());
  // ?
// ?

// --- 9. Update filtering for desired pose/orientation ---
//   {
//     std::lock_guard<std::mutex> lock(position_and_orientation_d_target_mutex_);
//     if (new_goal_pose_received_) {
//       position_d_ = (1.0 - filter_params_) * position_d_ + filter_params_ * position_d_target_;
//       orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_); //
//       new_goal_pose_received_ = false;
//     }
//   }

// ?  // Log the desired pose and orientation.
//   RCLCPP_INFO(get_node()->get_logger(), "Desired Pose: [x: %f, y: %f, z: %f]",
//   position_d_.x(), position_d_.y(), position_d_.z());
//   RCLCPP_INFO(get_node()->get_logger(), "Desired Orientation: [w: %f, x: %f, y: %f, z: %f]",
//   orientation_d_.w(), orientation_d_.x(), orientation_d_.y(), orientation_d_.z());
// ?

  // --- 10. Filter compliance parameters ---
  {
    std::lock_guard<std::mutex> lock(position_and_orientation_d_target_mutex_);
    if (new_goal_pose_received_) {
      cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
      cartesian_damping_   = filter_params_ * cartesian_damping_target_   + (1.0 - filter_params_) * cartesian_damping_;
      nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    }
  }
  return controller_interface::return_type::OK;
}


//!
//!

void CartesianImpedanceController::equilibriumPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    RCLCPP_INFO(get_node()->get_logger(), "Received new goal pose message.");
std::lock_guard<std::mutex> lock(position_and_orientation_d_target_mutex_);
position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z; // position_d_target_ 
Eigen::Quaterniond new_orientation(msg->pose.orientation.w, 
                                   msg->pose.orientation.x,
                                   msg->pose.orientation.y, 
                                   msg->pose.orientation.z);
orientation_d_ = updateOrientationTarget(orientation_d_, new_orientation); //orientation_d_target_ 
new_goal_pose_received_ = true;
}


CartesianImpedanceController::Vector7d CartesianImpedanceController::saturateTorqueRate(
    const Vector7d& tau_d_calculated,
    const Vector7d& tau_j_d) {
  Vector7d tau_d_saturated;
  for (size_t i = 0; i < N_JOINTS; i++) {
    double difference = tau_d_calculated[i] - tau_j_d[i];
    tau_d_saturated[i] = tau_j_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

} // namespace franka_teleop_pkg

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  franka_teleop_pkg::CartesianImpedanceController, 
  controller_interface::ControllerInterface
)