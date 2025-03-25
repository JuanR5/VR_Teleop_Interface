#include "vr_interface/controller_supervisor.hpp"

namespace franka_enhanced_controller {

ControllerSupervisor::ControllerSupervisor(const std::string& node_name)
    : Node(node_name) {
    
    // Initialize clients
    initializeRecoveryClient();
    initializeControllerClients();

    // Create monitoring timer (checks every 100ms)
    monitor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ControllerSupervisor::monitorCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Controller Supervisor initialized");
}

void ControllerSupervisor::initializeRecoveryClient() {
    error_recovery_client_ = rclcpp_action::create_client<franka_msgs::action::ErrorRecovery>(
        this,
        "/action_server/error_recovery"
    );

    // Wait for action server
    while (!error_recovery_client_->wait_for_action_server(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for error recovery action server...");
    }
}

void ControllerSupervisor::initializeControllerClients() {
    list_controllers_client_ = this->create_client<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers");
    
    switch_controller_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");

    while (!list_controllers_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for controller services");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for controller manager services...");
    }
}

void ControllerSupervisor::monitorCallback() {
    if (is_recovering_) {
        return;  // Skip monitoring while recovery is in progress
    }

    checkControllerState();
}

void ControllerSupervisor::checkControllerState() {
    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    
    auto future_result = list_controllers_client_->async_send_request(request);
    
    if (future_result.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
        auto result = future_result.get();
        bool controller_found = false;
        bool needs_recovery = false;

        for (const auto& controller : result->controller) {
            if (controller.name == controller_name_) {
                controller_found = true;
                if (controller.state == "inactive" || controller.state == "error") {
                    needs_recovery = true;
                }
                break;
            }
        }

        if (needs_recovery) {
            RCLCPP_WARN(get_logger(), "Controller needs recovery, initiating recovery sequence");
            handleControllerFailure();
        }
    }
}

bool ControllerSupervisor::canAttemptRecovery() {
    if (recovery_attempts_ >= MAX_RECOVERY_ATTEMPTS) {
        auto elapsed = this->now() - last_recovery_attempt_;
        if (elapsed > RECOVERY_COOLDOWN) {
            resetRecoveryCount();
            return true;
        }
        return false;
    }
    return true;
}

bool ControllerSupervisor::attemptRecovery() {
    if (!canAttemptRecovery()) {
        RCLCPP_WARN(this->get_logger(), 
            "Too many recovery attempts. Waiting for cooldown period.");
        return false;
    }

    is_recovering_ = true;
    recovery_attempts_++;
    last_recovery_attempt_ = this->now();

    auto goal_msg = franka_msgs::action::ErrorRecovery::Goal();
    
    auto send_goal_options = 
        rclcpp_action::Client<franka_msgs::action::ErrorRecovery>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const auto& goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Recovery goal rejected");
                is_recovering_ = false;
            }
        };

    send_goal_options.result_callback =
        [this](const auto& result) {
            is_recovering_ = false;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Recovery successful");
                restartController();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Recovery failed");
            }
        };

    error_recovery_client_->async_send_goal(goal_msg, send_goal_options);
    return true;
}

void ControllerSupervisor::restartController() {
    auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_request->stop_controllers = {controller_name_};
    switch_request->start_controllers = {controller_name_};
    switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    switch_request->start_asap = true;
    switch_request->timeout = {5, 0};  // 5 seconds timeout

    auto future_result = switch_controller_client_->async_send_request(switch_request);
    
    if (future_result.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        auto result = future_result.get();
        if (result->ok) {
            RCLCPP_INFO(get_logger(), "Controller successfully restarted");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to restart controller");
        }
    }
}

void ControllerSupervisor::resetRecoveryCount() {
    recovery_attempts_ = 0;
}

void ControllerSupervisor::handleControllerFailure() {
    RCLCPP_WARN(this->get_logger(), "Controller failure detected, attempting recovery");
    attemptRecovery();
}

} // namespace franka_enhanced_controller

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<franka_enhanced_controller::ControllerSupervisor>("controller_supervisor");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}