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

// Global serial port handle
int serial_port;

class MyBotaForceTorqueSensorComm : public BotaForceTorqueSensorComm
{
public:
  int serialReadBytes(uint8_t* data, size_t len) override {
    return read(serial_port, data, len);
  }
  int serialAvailable() override {
    int bytes;
    ioctl(serial_port, FIONREAD, &bytes);
    return bytes;
  }
};

class FTSensorNode : public rclcpp::Node
{
public:
  FTSensorNode() : Node("ft_sensor_node"), running_(true)
  {
    // Declare and get parameters
    this->declare_parameter("port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 460800);
    this->declare_parameter("frame_id", "ft_sensor_link");

    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // Open the serial port
    serial_port = open(port.c_str(), O_RDWR);
    if (serial_port < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error %i opening device: %s", errno, strerror(errno));
      if (errno == 13) {
        RCLCPP_ERROR(this->get_logger(), "Add the current user to the dialout group");
      }
      throw std::runtime_error("Failed to open serial port");
    }

    // Configure the serial port
    struct termios tty;
    struct serial_struct ser_info;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_port, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
      throw std::runtime_error("Failed to configure serial port");
    }

    tty.c_cflag &= ~PARENB;          // Disable parity
    tty.c_cflag &= ~CSTOPB;          // 1 stop bit
    tty.c_cflag |= CS8;              // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;         // Disable RTS/CTS
    tty.c_cflag |= CREAD | CLOCAL;   // Enable reading, ignore ctrl lines

    tty.c_lflag &= ~ICANON;          // Disable canonical mode
    tty.c_lflag &= ~ECHO;            // Disable echo
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;            // Disable signal chars
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;           // Prevent special output processing
    tty.c_oflag &= ~ONLCR;           // Prevent newline conversion

    tty.c_cc[VTIME] = 10;            // Wait up to 1 second
    tty.c_cc[VMIN] = 0;              // Return as soon as any data is available

    // Set baud rate (note: if using integer baudrate, ensure it corresponds to a valid constant)
    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
      throw std::runtime_error("Failed to configure serial port");
    }

    // Enable low latency mode for FTDI devices
    ioctl(serial_port, TIOCGSERIAL, &ser_info);
    ser_info.flags |= ASYNC_LOW_LATENCY;
    ioctl(serial_port, TIOCSSERIAL, &ser_info);

    // Flush any stale data
    tcflush(serial_port, TCIOFLUSH);

    // Create publisher for sensor data
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("ft_sensor/data", 10);

    // Start dedicated thread for sensor polling
    sensor_thread_ = std::thread(&FTSensorNode::sensorLoop, this);
  }

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
  void sensorLoop()
  {
    // Continuously poll the sensor similar to your working example
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
            // Publish valid sensor data immediately
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
          // Read one dummy byte to help regain sync
          if (sensor_.serialAvailable()) {
            uint8_t dummy;
            sensor_.serialReadBytes(&dummy, 1);
          }
          break;
        case BotaForceTorqueSensorComm::NO_FRAME:
          // No frame available; sleep briefly to avoid busy waiting
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          break;
      }
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
  MyBotaForceTorqueSensorComm sensor_;
  std::string frame_id_;
  std::thread sensor_thread_;
  std::atomic<bool> running_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FTSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
