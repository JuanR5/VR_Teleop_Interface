/**
 * @file ft_sensor_node.cpp
 * @brief ROS2 node for interfacing with a Bota Force Torque sensor over serial communication.
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "BotaForceTorqueSensorComm.h"

// Linux headers for serial communication
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <cstring>
#include <cstdint>
#include <thread>
#include <atomic>
#include <chrono>

// Global serial port file descriptor
int serial_port;

/**
 * @class MyBotaForceTorqueSensorComm
 * @brief Inherits from BotaForceTorqueSensorComm to provide platform-specific serial read operations.
 */
class MyBotaForceTorqueSensorComm : public BotaForceTorqueSensorComm
{
public:
  /**
   * @brief Reads bytes from the serial port.
   * @param data Pointer to buffer to store read data.
   * @param len Number of bytes to read.
   * @return Number of bytes read.
   */
  int serialReadBytes(uint8_t* data, size_t len) override {
    return read(serial_port, data, len);
  }

  /**
   * @brief Checks how many bytes are available to read.
   * @return Number of bytes available in the buffer.
   */
  int serialAvailable() override {
    int bytes;
    ioctl(serial_port, FIONREAD, &bytes);
    return bytes;
  }
};

/**
 * @class FTSensorNode
 * @brief Main ROS2 node class to manage the FT sensor lifecycle, communication, and data publishing.
 */
class FTSensorNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor that initializes the node, configures the serial port, and starts the polling thread.
   */
  FTSensorNode() : Node("ft_sensor_node"), running_(true)
  {
    // Declare parameters and retrieve them
    this->declare_parameter("port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 460800);
    this->declare_parameter("frame_id", "ft_sensor_link");

    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // Attempt to open serial port
    serial_port = open(port.c_str(), O_RDWR);
    if (serial_port < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error %i opening device: %s", errno, strerror(errno));
      if (errno == 13) {
        RCLCPP_ERROR(this->get_logger(), "Permission denied. Add the user to the 'dialout' group.");
      }
      throw std::runtime_error("Failed to open serial port");
    }

    // Configure serial communication settings
    struct termios tty;
    struct serial_struct ser_info;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_port, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
      throw std::runtime_error("Failed to get serial attributes");
    }

    tty.c_cflag &= ~PARENB;                    // No parity
    tty.c_cflag &= ~CSTOPB;                    // One stop bit
    tty.c_cflag |= CS8;                        // 8-bit characters
    tty.c_cflag &= ~CRTSCTS;                   // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;             // Enable receiver, ignore modem control lines

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG); // Raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);           // Raw output

    tty.c_cc[VTIME] = 10;                      // 1 second timeout
    tty.c_cc[VMIN] = 0;                        // No minimum read

    // ⚠️ POTENTIAL ISSUE: Ensure baudrate is supported constant (e.g., B460800)
    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
      throw std::runtime_error("Failed to set serial attributes");
    }

    // Enable low latency mode (specific to FTDI devices)
    ioctl(serial_port, TIOCGSERIAL, &ser_info);
    ser_info.flags |= ASYNC_LOW_LATENCY;
    ioctl(serial_port, TIOCSSERIAL, &ser_info);

    tcflush(serial_port, TCIOFLUSH); // Clear serial buffer

    // Set up ROS2 publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("ft_sensor/data", 10);

    // Start background thread to poll sensor
    sensor_thread_ = std::thread(&FTSensorNode::sensorLoop, this);
  }

  /**
   * @brief Destructor that safely shuts down the background thread and closes the serial port.
   */
  ~FTSensorNode()
  {
    running_ = false;
    if (sensor_thread_.joinable()) {
      sensor_thread_.join();
    }
    close(serial_port);
    RCLCPP_INFO(this->get_logger(), "Serial port closed");
  }

private:
  /**
   * @brief Continuously polls sensor and publishes valid data over a ROS2 topic.
   */
  void sensorLoop()
  {
    while (rclcpp::ok() && running_) {
      int frameResult = sensor_.readFrame();

      switch (frameResult) {
        case BotaForceTorqueSensorComm::VALID_FRAME:
          if (sensor_.frame.data.status.val > 0) {
            RCLCPP_WARN(this->get_logger(), 
              "No valid forces: app_took_too_long: %i, overrange: %i, invalid_measurements: %i, raw_measurements: %i", 
              sensor_.frame.data.status.app_took_too_long,
              sensor_.frame.data.status.overrange,
              sensor_.frame.data.status.invalid_measurements,
              sensor_.frame.data.status.raw_measurements);
          } else {
            geometry_msgs::msg::Wrench wrench_msg;
            wrench_msg.force.x = sensor_.frame.data.forces[0];
            wrench_msg.force.y = sensor_.frame.data.forces[1];
            wrench_msg.force.z = sensor_.frame.data.forces[2];
            wrench_msg.torque.x = sensor_.frame.data.forces[3];
            wrench_msg.torque.y = sensor_.frame.data.forces[4];
            wrench_msg.torque.z = sensor_.frame.data.forces[5];
            publisher_->publish(wrench_msg);
            RCLCPP_WARN(this->get_logger(), "Publishing sensor data");
          }
          break;

        case BotaForceTorqueSensorComm::NOT_VALID_FRAME:
          RCLCPP_WARN(this->get_logger(), "No valid frame: CRC error count: %i", sensor_.get_crc_count());
          break;

        case BotaForceTorqueSensorComm::NOT_ALLIGNED_FRAME:
          RCLCPP_WARN(this->get_logger(), "Lost sync, trying to reconnect");
          if (sensor_.serialAvailable()) {
            uint8_t dummy;
            sensor_.serialReadBytes(&dummy, 1); // Flush one byte to regain sync
          }
          break;

        case BotaForceTorqueSensorComm::NO_FRAME:
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          break;
      }
    }
  }

  // ROS2 publisher for wrench data
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;

  // Sensor communication interface
  MyBotaForceTorqueSensorComm sensor_;

  // Frame ID used in message headers (not currently published)
  std::string frame_id_;

  // Background thread for polling the sensor
  std::thread sensor_thread_;

  // Atomic flag to control thread lifecycle
  std::atomic<bool> running_;
};

/**
 * @brief Main function initializes the ROS2 system and spins the sensor node.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FTSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
