#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <franka_msgs/action/error_recovery.hpp>
#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <chrono>

namespace franka_enhanced_controller {

class ControllerSupervisor : public rclcpp::Node {
public:
    explicit ControllerSupervisor(const std::string& node_name);

private:
    // Constants for recovery attempts
    static constexpr int MAX_RECOVERY_ATTEMPTS = 3;
    static constexpr std::chrono::seconds RECOVERY_COOLDOWN{5};
    
    // Recovery action client
    rclcpp_action::Client<franka_msgs::action::ErrorRecovery>::SharedPtr error_recovery_client_;
    
    // State tracking
    int recovery_attempts_ = 0;
    rclcpp::Time last_recovery_attempt_;
    bool is_recovering_ = false;

    // Methods
    void initializeRecoveryClient();
    bool attemptRecovery();
    void resetRecoveryCount();
    bool canAttemptRecovery();
    
    // Timer for monitoring
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    void monitorCallback();

    // Controller state monitoring
    void checkControllerState();
    void handleControllerFailure();
};

} // namespace franka_enhanced_controller