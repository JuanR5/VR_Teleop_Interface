/**
 * @file ft_filter_node.cpp
 * @brief ROS2 Node for filtering force-torque sensor data and publishing intensity/frequency values.
 */

#include "ft_filter_node.hpp"

/**
 * @brief Construct the FTFilterNode, initializing parameters, subscribers, and publishers.
 */
FTFilterNode::FTFilterNode() : Node("ft_filter_node"), is_calibrating_(true), filter_window_size_(10)
{
    // Declare and retrieve ROS parameters
    this->declare_parameter("max_range", 2.0);             ///< Max expected value for intensity normalization
    this->declare_parameter("calibration_time", 5.0);      ///< Duration of sensor calibration in seconds

    max_range_ = this->get_parameter("max_range").as_double();
    calibration_time_ = this->get_parameter("calibration_time").as_double();

    // Record calibration start time
    calibration_start_time_ = this->now();
    
    // Subscribe to raw force-torque sensor data
    subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "ft_sensor/data", 10, std::bind(&FTFilterNode::ftCallback, this, std::placeholders::_1));

    // Publisher for processed output: intensity and frequency
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("rumble_output", 10);

    RCLCPP_INFO(this->get_logger(), "FT Filter Node initialized. Calibrating for %.1f seconds...", calibration_time_);
}

/**
 * @brief Callback triggered upon receiving sensor data.
 * Applies filtering, computes intensity/frequency, and publishes result.
 *
 * @param msg Pointer to received Wrench message.
 */
void FTFilterNode::ftCallback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    // Extract force and torque values
    std::vector<double> forces = {msg->force.x, msg->force.y, msg->force.z};
    std::vector<double> torques = {msg->torque.x, msg->torque.y, msg->torque.z};

    // Calibration phase
    if (is_calibrating_)
    {
        calibrateSensor(forces, torques);
        return;
    }
    
    // Apply moving average filtering
    auto filtered_forces = applyMovingAverage(filter_buffer_forces_, forces);
    auto filtered_torques = applyMovingAverage(filter_buffer_torques_, torques);

    // Compute normalized intensity and determine frequency
    double intensity = computeIntensity(filtered_forces, filtered_torques);
    double frequency = (intensity > (2.0 / 3.0)) ? 1.0 : 0.3;

    // Round values for clarity
    intensity = roundToDecimal(intensity, 3);
    frequency = roundToDecimal(frequency, 3);

    if (intensity > 0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Force increase detected in variable: %s", 
                    getMaxIntensityVariable(filtered_forces, filtered_torques).c_str());
    }

    // Publish final output
    publishOutput(intensity, frequency);
}

/**
 * @brief Collects data for calibration and calculates the reference baseline.
 *
 * @param forces Current force readings.
 * @param torques Current torque readings.
 */
void FTFilterNode::calibrateSensor(const std::vector<double>& forces, const std::vector<double>& torques)
{
    calibration_data_forces_.push_back(forces);
    calibration_data_torques_.push_back(torques);

    if ((this->now() - calibration_start_time_).seconds() >= calibration_time_)
    {
        is_calibrating_ = false;
        reference_forces_ = computeAverage(calibration_data_forces_);
        reference_torques_ = computeAverage(calibration_data_torques_);
        RCLCPP_INFO(this->get_logger(), "Calibration complete.");
    }
}

/**
 * @brief Applies a moving average filter to incoming data.
 *
 * @param buffer Reference to buffer holding historical data.
 * @param new_values New incoming values.
 * @return Smoothed output vector.
 */
std::vector<double> FTFilterNode::applyMovingAverage(std::vector<std::vector<double>>& buffer, const std::vector<double>& new_values)
{
    buffer.push_back(new_values);
    if (buffer.size() > filter_window_size_)
    {
        buffer.erase(buffer.begin());
    }
    return computeAverage(buffer);
}

/**
 * @brief Computes the average of a set of vectors.
 *
 * @param data Vector of vectors to average.
 * @return Resulting average vector.
 */
std::vector<double> FTFilterNode::computeAverage(const std::vector<std::vector<double>>& data)
{
    size_t size = data.front().size();
    std::vector<double> avg(size, 0.0);

    for (const auto& sample : data)
    {
        for (size_t i = 0; i < size; ++i)
        {
            avg[i] += sample[i];
        }
    }

    for (double& val : avg)
    {
        val /= data.size();
    }

    return avg;
}

/**
 * @brief Computes intensity as the maximum normalized deviation from the reference.
 *
 * @param forces Filtered forces.
 * @param torques Filtered torques.
 * @return Clamped intensity value [0.0, 1.0].
 */
double FTFilterNode::computeIntensity(const std::vector<double>& forces, const std::vector<double>& torques)
{
    double max_intensity = 0.0;

    for (size_t i = 0; i < forces.size(); ++i)
    {
        double intensity = std::abs(forces[i] - reference_forces_[i]) / max_range_;
        max_intensity = std::max(max_intensity, intensity);
    }

    for (size_t i = 0; i < torques.size(); ++i)
    {
        double intensity = std::abs(torques[i] - reference_torques_[i]) / max_range_;
        max_intensity = std::max(max_intensity, intensity);
    }

    return std::clamp(max_intensity, 0.0, 1.0);
}

/**
 * @brief Identifies the component (Fx, Fy, Fz, Tx, Ty, Tz) with the highest deviation.
 *
 * @param forces Filtered forces.
 * @param torques Filtered torques.
 * @return String label of the most intense variable.
 */
std::string FTFilterNode::getMaxIntensityVariable(const std::vector<double>& forces, const std::vector<double>& torques)
{
    const std::vector<std::string> labels = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};

    std::vector<double> values = forces;
    values.insert(values.end(), torques.begin(), torques.end());

    size_t max_idx = std::distance(values.begin(), std::max_element(values.begin(), values.end()));
    return labels[max_idx];
}

/**
 * @brief Publishes the final intensity and frequency to a Float32MultiArray message.
 *
 * @param intensity Normalized intensity value.
 * @param frequency Calculated output frequency.
 */
void FTFilterNode::publishOutput(double intensity, double frequency)
{
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {static_cast<float>(intensity), static_cast<float>(frequency)};
    publisher_->publish(msg);
}

/**
 * @brief Rounds a value to a given number of decimal places.
 *
 * @param value The input value.
 * @param decimal_places Number of digits to round to.
 * @return Rounded value.
 */
double FTFilterNode::roundToDecimal(double value, int decimal_places)
{
    double multiplier = std::pow(10.0, decimal_places);
    return std::round(value * multiplier) / multiplier;
}

/**
 * @brief Entry point of the ROS2 node.
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FTFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
