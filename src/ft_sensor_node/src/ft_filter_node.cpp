/**
 * @file ft_filter_node.cpp
 * @brief ROS2 Node for filtering force-torque sensor data and publishing intensity/frequency values.
 */

 #include "ft_filter_node.hpp"

 FTFilterNode::FTFilterNode() : Node("ft_filter_node"), is_calibrating_(true)
 {
     // Declare and retrieve ROS parameters with defaults
     this->declare_parameter("max_range", 6.0);             // Max expected value for intensity normalization
     this->declare_parameter("calibration_time", 5.0);        // Duration of sensor calibration in seconds
     this->declare_parameter("filter_window_size", 50);       // Window size for moving average filter
     this->declare_parameter("detection_threshold", 0.67);    // Threshold to determine frequency output
     this->declare_parameter("high_frequency", 1.0);          // Frequency when intensity exceeds threshold
     this->declare_parameter("low_frequency", 0.3);           // Frequency when intensity is below threshold
 
     // New parameters for improved collision detection
     this->declare_parameter("force_weight", 0.8);            // Weight for force deviation in collision detection
     this->declare_parameter("torque_weight", 0.2);           // Weight for torque deviation in collision detection
     this->declare_parameter("torque_deadband", 0.1);         // Minimum torque deviation to consider
 
     max_range_ = this->get_parameter("max_range").as_double();
     calibration_time_ = this->get_parameter("calibration_time").as_double();
     filter_window_size_ = this->get_parameter("filter_window_size").as_int();
     detection_threshold_ = this->get_parameter("detection_threshold").as_double();
     high_frequency_ = this->get_parameter("high_frequency").as_double();
     low_frequency_ = this->get_parameter("low_frequency").as_double();
 
     force_weight_ = this->get_parameter("force_weight").as_double();
     torque_weight_ = this->get_parameter("torque_weight").as_double();
     torque_deadband_ = this->get_parameter("torque_deadband").as_double();
 
     // Record calibration start time
     calibration_start_time_ = this->now();
     
     // Subscribe to raw force-torque sensor data
     subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>(
         "ft_sensor/data", 10, std::bind(&FTFilterNode::ftCallback, this, std::placeholders::_1));
 
     // Publisher for processed output: intensity and frequency
     publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("rumble_output", 10);
 
     RCLCPP_INFO(this->get_logger(), "FT Filter Node initialized. Calibrating for %.1f seconds...", calibration_time_);
 }
 
 void FTFilterNode::ftCallback(const geometry_msgs::msg::Wrench::SharedPtr msg)
 {
     // Extract force and torque values
     std::vector<double> forces = {msg->force.x, msg->force.y, msg->force.z};
     std::vector<double> torques = {msg->torque.x, msg->torque.y, msg->torque.z};
 
     // Calibration phase: collect baseline data
     if (is_calibrating_)
     {
         calibrateSensor(forces, torques);
         return;
     }
     
     // Apply moving average filtering
     auto filtered_forces = applyMovingAverage(filter_buffer_forces_, forces);
     auto filtered_torques = applyMovingAverage(filter_buffer_torques_, torques);
 
     // Compute normalized intensity and determine frequency based on configurable threshold
     double intensity = computeIntensity(filtered_forces, filtered_torques);
     double frequency = (intensity > detection_threshold_) ? high_frequency_ : low_frequency_;
 
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
 
 std::vector<double> FTFilterNode::applyMovingAverage(std::vector<std::vector<double>>& buffer, const std::vector<double>& new_values)
 {
     buffer.push_back(new_values);
     if (buffer.size() > static_cast<size_t>(filter_window_size_))
     {
         buffer.erase(buffer.begin());
     }
     return computeAverage(buffer);
 }
 
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
  * @brief Computes a weighted collision intensity based on Euclidean deviation in forces and torques.
  *        Torque deviations below a specified deadband are ignored.
  */
 double FTFilterNode::computeIntensity(const std::vector<double>& forces, const std::vector<double>& torques)
 {
     // Compute Euclidean norm for force deviations
     double force_sum = 0.0;
     for (size_t i = 0; i < forces.size(); ++i)
     {
          double diff = forces[i] - reference_forces_[i];
          force_sum += diff * diff;
     }
     double force_intensity = std::sqrt(force_sum) / max_range_;
 
     // Compute Euclidean norm for torque deviations
     double torque_sum = 0.0;
     for (size_t i = 0; i < torques.size(); ++i)
     {
          double diff = torques[i] - reference_torques_[i];
          torque_sum += diff * diff;
     }
     double torque_intensity = std::sqrt(torque_sum) / max_range_;
 
     // Apply deadband: ignore small torque deviations
     if (torque_intensity < torque_deadband_)
     {
          torque_intensity = 0.0;
     }
 
     // Combine using weights
     double combined_intensity = force_weight_ * force_intensity + torque_weight_ * torque_intensity;
 
     return std::clamp(combined_intensity, 0.0, 1.0);
 }
 
 std::string FTFilterNode::getMaxIntensityVariable(const std::vector<double>& forces, const std::vector<double>& torques)
 {
     const std::vector<std::string> labels = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
 
     std::vector<double> values = forces;
     values.insert(values.end(), torques.begin(), torques.end());
 
     size_t max_idx = std::distance(values.begin(), std::max_element(values.begin(), values.end()));
     return labels[max_idx];
 }
 
 void FTFilterNode::publishOutput(double intensity, double frequency)
 {
     std_msgs::msg::Float32MultiArray msg;
     msg.data = {static_cast<float>(intensity), static_cast<float>(frequency)};
     publisher_->publish(msg);
 }
 
 double FTFilterNode::roundToDecimal(double value, int decimal_places)
 {
     double multiplier = std::pow(10.0, decimal_places);
     return std::round(value * multiplier) / multiplier;
 }
 
 int main(int argc, char** argv)
 {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<FTFilterNode>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
 }
 