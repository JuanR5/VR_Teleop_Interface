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
 * Computes the rotation error as desired * current.inverse().
 *
 * @param current The current orientation.
 * @param desired The desired orientation.
 * @return Error quaternion.
 */
static inline Eigen::Quaterniond computeQuaternionError(const Eigen::Quaterniond &current, const Eigen::Quaterniond &desired) {
  return desired * current.inverse();
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
static inline Eigen::Quaterniond updateOrientationTarget(const Eigen::Quaterniond &current_target, const Eigen::Quaterniond &new_target) {
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
  std::string arm_prefix = (arm_id_.empty() ? "fr3" : arm_id_) + "_joint";
  for (size_t index = 1; index <= N_JOINTS; index++) {
    config.names.push_back(arm_prefix + std::to_string(index) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration 
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  std::string arm_prefix = (arm_id_.empty() ? "fr3" : arm_id_) + "_joint";
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
    // Declare parameters.
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<double>("velocity_filter_alpha_gain", 0.99);
    auto_declare<std::string>("cmd_topic", "~/cmd_pose");
    auto_declare<std::string>("state_topic", "~/commanded_state");
    auto_declare<std::string>("arm_id", "fr3");
    // Parameterize compliance gains.
    auto_declare<double>("translational_stiffness", 150.0);
    auto_declare<double>("rotational_stiffness", 10.0);
    
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
  velocity_filter_alpha_gain_ = get_node()->get_parameter("velocity_filter_alpha_gain").as_double();

  // Set up equilibrium pose subscription.
  try {
    sub_equilibrium_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "equilibrium_pose", 20,
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
  velocity_filter_alpha_gain_ = 0.0;
  for (size_t index = 0; index < N_JOINTS; index++) {
    k_gains_(static_cast<int>(index)) = 0;
    d_gains_(static_cast<int>(index)) = 0;
  }
  subscriber_.reset();
  sub_equilibrium_pose_.reset();
  publisher_.reset();
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
  Eigen::Vector3d position_init(init_robot_state_.o_t_ee.pose.position.x,
                                init_robot_state_.o_t_ee.pose.position.y,
                                init_robot_state_.o_t_ee.pose.position.z);
  Eigen::Quaterniond orientation_init(
      init_robot_state_.o_t_ee.pose.orientation.w, 
      init_robot_state_.o_t_ee.pose.orientation.x,
      init_robot_state_.o_t_ee.pose.orientation.y, 
      init_robot_state_.o_t_ee.pose.orientation.z);
      
  Eigen::Affine3d initial_transform = Eigen::Affine3d::Identity();
  initial_transform.translation() = position_init;
  initial_transform.rotate(orientation_init.toRotationMatrix());
  
  position_d_ = initial_transform.translation();
  orientation_d_ = initial_transform.rotation();
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = initial_transform.rotation();

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

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
  // --- 1. Acquire current robot state ---
  robot_state_ = franka_msgs::msg::FrankaRobotState();
  franka_robot_state_->get_values_as_message(robot_state_);

  // --- 2. Retrieve robot model data (fixed-size arrays) ---
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);

  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // --- 3. Map current joint state (positions, velocities, efforts) ---
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
      
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() = position;
  transform.rotate(orientation.toRotationMatrix());

  // --- 5. Compute Cartesian error ---
  Eigen::Matrix<double, 6, 1> position_error;
  position_error.head(3) = position - position_d_;
  
  orientation = normalizeQuaternion(orientation);
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() = -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion = computeQuaternionError(orientation, orientation_d_);
  position_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  position_error.tail(3) = -transform.rotation() * position_error.tail(3);

  // --- 6. Compute control torques ---
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);
  Eigen::MatrixXd jacobian_transpose_pinv = pseudoInverse(jacobian.transpose(), true);
  tau_task = jacobian.transpose() * (-cartesian_stiffness_ * position_error - cartesian_damping_ * (jacobian * dq));
  tau_nullspace = (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
                  (nullspace_stiffness_ * (q_d_nullspace_ - q) - (2.0 * sqrt(nullspace_stiffness_)) * dq);
  tau_d = tau_task + tau_nullspace + coriolis;
  
  tau_d = saturateTorqueRate(tau_d, tau_j_d);

  // --- 7. Send computed torques ---
  for (int i = 0; i < num_joints; i++) {
    command_interfaces_[i].set_value(tau_d[i]);
  }

  // --- 8. Update filtering and target values ---
  cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  {
    std::lock_guard<std::mutex> lock(position_and_orientation_d_target_mutex_);
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  }

  // --- 9. Publish commanded state (preallocate vector if needed) ---
  if (realtime_publisher_ && realtime_publisher_->trylock()) {
    realtime_publisher_->msg_.header.stamp = time;
    if (realtime_publisher_->msg_.data.size() != N_JOINTS) {
      realtime_publisher_->msg_.data.resize(N_JOINTS, 0.0);
    }
    for (size_t i = 0; i < N_JOINTS; ++i) {
      realtime_publisher_->msg_.data[i] = tau_d[i];
    }
    realtime_publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

void CartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond new_orientation(msg->pose.orientation.w, 
                                     msg->pose.orientation.x,
                                     msg->pose.orientation.y, 
                                     msg->pose.orientation.z);
  orientation_d_target_ = updateOrientationTarget(orientation_d_target_, new_orientation);
}

CartesianImpedanceController::Vector7d CartesianImpedanceController::saturateTorqueRate(
    const Vector7d& tau_d_calculated,
    const Vector7d& tau_j_d) {
  Vector7d tau_d_saturated;
  for (size_t i = 0; i < 7; i++) {
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
