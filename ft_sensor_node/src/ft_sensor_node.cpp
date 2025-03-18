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
    FTSensorNode() : Node("ft_sensor_node")
    {
        // Declare parameters
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 460800);
        this->declare_parameter("frame_id", "ft_sensor_link");

        // Get parameters
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Initialize serial port
        serial_port = open(port.c_str(), O_RDWR);
        if (serial_port < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i opening device: %s", errno, strerror(errno));
            if (errno == 13) {
                RCLCPP_ERROR(this->get_logger(), "Add the current user to the dialout group");
            }
            throw std::runtime_error("Failed to open serial port");
        }

        // Configure serial port
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_port, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s", errno, strerror(errno));
            throw std::runtime_error("Failed to configure serial port");
        }

        tty.c_cflag &= ~PARENB; // Disable parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag |= CS8; // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

        tty.c_lflag &= ~ICANON; // Disable canonical mode
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT, SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes
        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to CR/LF
        tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds)
        tty.c_cc[VMIN] = 0;

        // Set baud rate
        cfsetispeed(&tty, baudrate);
        cfsetospeed(&tty, baudrate);

        // Save settings
        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s", errno, strerror(errno));
            throw std::runtime_error("Failed to configure serial port");
        }

        // Enable low latency mode
        struct serial_struct ser_info;
        ioctl(serial_port, TIOCGSERIAL, &ser_info);
        ser_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(serial_port, TIOCSSERIAL, &ser_info);

        // Create a publisher for the force-torque data
        publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("ft_sensor/data", 10);

        // Create a timer to read and publish data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 100 Hz
            std::bind(&FTSensorNode::readAndPublishData, this));
    }

    ~FTSensorNode()
    {
        close(serial_port);
        RCLCPP_INFO(this->get_logger(), "Serial port closed");
    }

private:
    void readAndPublishData()
    {
        switch (sensor_.readFrame())
        {
            case BotaForceTorqueSensorComm::VALID_FRAME:
                if (sensor_.frame.data.status.val > 0)
                {
                    RCLCPP_WARN(this->get_logger(), "No valid forces:");
                    RCLCPP_WARN(this->get_logger(), " app_took_too_long: %i", sensor_.frame.data.status.app_took_too_long);
                    RCLCPP_WARN(this->get_logger(), " overrange: %i", sensor_.frame.data.status.overrange);
                    RCLCPP_WARN(this->get_logger(), " invalid_measurements: %i", sensor_.frame.data.status.invalid_measurements);
                    RCLCPP_WARN(this->get_logger(), " raw_measurements: %i", sensor_.frame.data.status.raw_measurements);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Received valid frame:");
                    RCLCPP_INFO(this->get_logger(), " Forces: [%f, %f, %f]", sensor_.frame.data.forces[0], sensor_.frame.data.forces[1], sensor_.frame.data.forces[2]);
                    RCLCPP_INFO(this->get_logger(), " Torques: [%f, %f, %f]", sensor_.frame.data.forces[3], sensor_.frame.data.forces[4], sensor_.frame.data.forces[5]);

                    // Publish the force-torque data
                    auto wrench_msg = geometry_msgs::msg::Wrench();
                    wrench_msg.force.x = sensor_.frame.data.forces[0];
                    wrench_msg.force.y = sensor_.frame.data.forces[1];
                    wrench_msg.force.z = sensor_.frame.data.forces[2];
                    wrench_msg.torque.x = sensor_.frame.data.forces[3];
                    wrench_msg.torque.y = sensor_.frame.data.forces[4];
                    wrench_msg.torque.z = sensor_.frame.data.forces[5];
                    publisher_->publish(wrench_msg);
                }
                break;
            case BotaForceTorqueSensorComm::NOT_VALID_FRAME:
                RCLCPP_WARN(this->get_logger(), "No valid frame: CRC error count: %i", sensor_.get_crc_count());
                break;
            case BotaForceTorqueSensorComm::NOT_ALLIGNED_FRAME:
                RCLCPP_WARN(this->get_logger(), "Lost sync, trying to reconnect");
                break;
            case BotaForceTorqueSensorComm::NO_FRAME:
                RCLCPP_DEBUG(this->get_logger(), "No frame received");
                break;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    MyBotaForceTorqueSensorComm sensor_;
    std::string frame_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FTSensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}