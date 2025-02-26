#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <franka/exception.h>
#include <Eigen/Dense>
#include <mutex>

class FrankaMovementController : public rclcpp::Node {
public:
    FrankaMovementController()
        : Node("franka_movement_controller") {
        // Declare parameters
        this->declare_parameter<double>("step_size", 0.05);
        this->declare_parameter<int>("max_recovery_attempts", 10);

        // Get parameters
        step_size_ = this->get_parameter("step_size").as_double();
        max_recovery_attempts_ = this->get_parameter("max_recovery_attempts").as_int();

        // Initialize subscription
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "controller_movement", 10,
            std::bind(&FrankaMovementController::movementCallback, this, std::placeholders::_1));

        // Initialize the Cartesian Pose Interface
        franka_controller_ = std::make_shared<franka_semantic_components::FrankaCartesianPoseInterface>(true);
        if (!franka_controller_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize Franka Cartesian Pose Interface!");
            throw std::runtime_error("Failed to initialize Franka Cartesian Pose Interface");
        }

        RCLCPP_INFO(this->get_logger(), "FrankaMovementController initialized.");
    }

private:
    void movementCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_); // Ensure thread safety

        // Check if message is valid
        if (!msg) {
            RCLCPP_ERROR(this->get_logger(), "Received a null message!");
            return;
        }

        // Check if controller is initialized
        if (!franka_controller_) {
            RCLCPP_ERROR(this->get_logger(), "Controller not initialized. Retrying...");
            return;
        }

        // Validate message size
        if (msg->data.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "Invalid message size: Expected 6, got %zu", msg->data.size());
            return;
        }

        try {
            processMovement(msg);
        } catch (const franka::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Franka exception occurred: %s", e.what());
            RCLCPP_INFO(this->get_logger(), "Running error recovery...");
            try {
                runErrorRecovery();
            } catch (const std::exception& recovery_error) {
                RCLCPP_FATAL(this->get_logger(), "Error recovery failed: %s", recovery_error.what());
            }
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "Other exception occurred: %s", e.what());
        }
    }

    void processMovement(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Processing movement command...");

        double delta_x = msg->data[0];
        double delta_y = msg->data[1];
        double delta_z = msg->data[2];
        double delta_R = msg->data[3];
        double delta_P = msg->data[4];
        double delta_Y = msg->data[5];

        Eigen::Quaterniond orientation;
        Eigen::Vector3d position;

        try {
            std::tie(orientation, position) = franka_controller_->getCurrentOrientationAndTranslation();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in getCurrentOrientationAndTranslation: %s", e.what());
            throw;
        }

        // Debug print: Current pose
        RCLCPP_INFO(this->get_logger(), "Current Pose - Position: [%f, %f, %f], Orientation: [%f, %f, %f, %f]",
                    position(0), position(1), position(2),
                    orientation.x(), orientation.y(), orientation.z(), orientation.w());

        // Update position and orientation
        position(0) += delta_x * step_size_;
        position(1) += delta_y * step_size_;
        position(2) += delta_z * step_size_;

        Eigen::AngleAxisd rollAngle(delta_R * step_size_, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(delta_P * step_size_, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(delta_Y * step_size_, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond delta_orientation = yawAngle * pitchAngle * rollAngle;
        orientation = delta_orientation * orientation;

        // Debug print: New pose
        RCLCPP_INFO(this->get_logger(), "New Pose - Position: [%f, %f, %f], Orientation: [%f, %f, %f, %f]",
                    position(0), position(1), position(2),
                    orientation.x(), orientation.y(), orientation.z(), orientation.w());

        // Send new command
        if (!franka_controller_->setCommand(orientation, position)) {
            throw std::runtime_error("Failed to set command to Franka arm.");
        }
    }

    void runErrorRecovery() {
        RCLCPP_INFO(this->get_logger(), "Starting automatic error recovery...");

        for (int attempt = 1; attempt <= max_recovery_attempts_; ++attempt) {
            try {
                RCLCPP_INFO(this->get_logger(), "Recovery attempt %d of %d", attempt, max_recovery_attempts_);

                // Add a small delay before recovery attempt
                rclcpp::sleep_for(std::chrono::seconds(1));

                // Custom recovery logic (e.g., reinitialize controller)
                franka_controller_ = std::make_shared<franka_semantic_components::FrankaCartesianPoseInterface>(true);
                RCLCPP_INFO(this->get_logger(), "Error recovery successful!");
                return;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Recovery attempt %d failed: %s", attempt, e.what());

                if (attempt == max_recovery_attempts_) {
                    RCLCPP_FATAL(this->get_logger(), "All recovery attempts failed. Manual intervention required.");
                    throw;
                }
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    std::shared_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_controller_;
    double step_size_;
    int max_recovery_attempts_;
    std::mutex mutex_; // Mutex for thread safety
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaMovementController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}