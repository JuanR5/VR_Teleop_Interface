#ifndef FT_FILTER_NODE_HPP
#define FT_FILTER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include <numeric>
#include <cmath>
#include <string>

/**
 * @class FTFilterNode
 * @brief A ROS2 node that filters force-torque data and computes vibration intensity and frequency.
 */
class FTFilterNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor. Initializes parameters, subscribers, publishers, and calibration.
     */
    FTFilterNode();

private:
    /**
     * @brief Callback triggered when a new Wrench message is received.
     * @param msg Pointer to the received message.
     */
    void ftCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);

    /**
     * @brief Collects samples during calibration to establish baseline reference values.
     * @param forces Current force readings.
     * @param torques Current torque readings.
     */
    void calibrateSensor(const std::vector<double>& forces, const std::vector<double>& torques);

    /**
     * @brief Applies a moving average filter to smooth incoming data.
     * @param buffer Buffer of historical values.
     * @param new_values Latest input vector to be added to buffer.
     * @return Smoothed vector result.
     */
    std::vector<double> applyMovingAverage(std::vector<std::vector<double>>& buffer, const std::vector<double>& new_values);

    /**
     * @brief Computes the average of a list of vectors.
     * @param data Vector of vectors containing numeric samples.
     * @return Averaged result.
     */
    std::vector<double> computeAverage(const std::vector<std::vector<double>>& data);

    /**
     * @brief Computes the maximum normalized intensity based on deviation from calibrated reference.
     * @param forces Filtered force values.
     * @param torques Filtered torque values.
     * @return Intensity value in range [0.0, 1.0].
     */
    double computeIntensity(const std::vector<double>& forces, const std::vector<double>& torques);

    /**
     * @brief Identifies the force or torque variable contributing the most to the intensity.
     * @param forces Filtered forces.
     * @param torques Filtered torques.
     * @return Label of the variable with maximum deviation (e.g., "Fx", "Ty").
     */
    std::string getMaxIntensityVariable(const std::vector<double>& forces, const std::vector<double>& torques);

    /**
     * @brief Publishes computed intensity and frequency.
     * @param intensity Normalized magnitude.
     * @param frequency Output frequency.
     */
    void publishOutput(double intensity, double frequency);

    /**
     * @brief Rounds a floating-point number to the given number of decimal places.
     * @param value Value to round.
     * @param decimal_places Number of digits to keep.
     * @return Rounded result.
     */
    double roundToDecimal(double value, int decimal_places);

    // === ROS2 Communication ===

    /// Subscriber to raw Wrench sensor data
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr subscriber_;

    /// Publisher for processed vibration intensity and frequency
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    // === Parameters and Calibration ===

    /// Maximum expected force/torque range for normalization
    double max_range_;

    /// Duration of initial calibration in seconds
    double calibration_time_;

    /// Timestamp when calibration started
    rclcpp::Time calibration_start_time_;

    /// Flag indicating whether calibration is in progress
    bool is_calibrating_;

    /// Size of the moving average filter window
    size_t filter_window_size_;

    /// Threshold to determine frequency output based on intensity
    double detection_threshold_;

    /// Frequency output when intensity exceeds threshold
    double high_frequency_;

    /// Frequency output when intensity is below threshold
    double low_frequency_;

    /// Weight for force deviation in collision detection
    double force_weight_;

    /// Weight for torque deviation in collision detection
    double torque_weight_;

    /// Minimum torque deviation to consider (deadband)
    double torque_deadband_;

    // === Calibration Buffers ===

    /// Samples collected for force calibration
    std::vector<std::vector<double>> calibration_data_forces_;

    /// Samples collected for torque calibration
    std::vector<std::vector<double>> calibration_data_torques_;

    /// Reference force vector post-calibration
    std::vector<double> reference_forces_ = {0.0, 0.0, 0.0};

    /// Reference torque vector post-calibration
    std::vector<double> reference_torques_ = {0.0, 0.0, 0.0};

    // === Filter Buffers ===

    /// Circular buffer for force smoothing
    std::vector<std::vector<double>> filter_buffer_forces_;

    /// Circular buffer for torque smoothing
    std::vector<std::vector<double>> filter_buffer_torques_;
};

#endif // FT_FILTER_NODE_HPP
