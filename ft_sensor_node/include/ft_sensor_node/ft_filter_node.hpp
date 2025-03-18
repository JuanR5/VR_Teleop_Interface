#ifndef FT_FILTER_NODE_HPP
#define FT_FILTER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include <numeric>
#include <cmath>
#include <string>

class FTFilterNode : public rclcpp::Node
{
public:
    FTFilterNode();

private:
    void ftCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);
    void calibrateSensor(const std::vector<double>& forces, const std::vector<double>& torques);
    std::vector<double> applyMovingAverage(std::vector<std::vector<double>>& buffer, const std::vector<double>& new_values);
    std::vector<double> computeAverage(const std::vector<std::vector<double>>& data);
    double computeIntensity(const std::vector<double>& forces, const std::vector<double>& torques);
    std::string getMaxIntensityVariable(const std::vector<double>& forces, const std::vector<double>& torques);
    void publishOutput(double intensity, double frequency);
    double roundToDecimal(double value, int decimal_places);

    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    double max_range_; // Maximum range for intensity calculation
    double calibration_time_;
    rclcpp::Time calibration_start_time_;
    bool is_calibrating_;
    size_t filter_window_size_;
    std::vector<std::vector<double>> calibration_data_forces_;
    std::vector<std::vector<double>> calibration_data_torques_;
    std::vector<double> reference_forces_ = {0.0, 0.0, 0.0};
    std::vector<double> reference_torques_ = {0.0, 0.0, 0.0};
    std::vector<std::vector<double>> filter_buffer_forces_;
    std::vector<std::vector<double>> filter_buffer_torques_;
};

#endif // FT_FILTER_NODE_HPP