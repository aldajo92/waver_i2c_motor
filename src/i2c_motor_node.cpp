#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <linux/i2c-dev.h>

#define I2C_BUS "/dev/i2c-1"
#define I2C_ADDR 0x11

using namespace rclcpp;

class MotorController : public Node {
public:
    MotorController() : Node("motor_controller"), file_(-1) {
        file_ = open(I2C_BUS, O_RDWR);
        if (file_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open the I2C bus.");
            shutdown();
            return;
        }

        if (ioctl(file_, I2C_SLAVE, I2C_ADDR) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to talk to the I2C slave device.");
            close(file_);
            shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "I2C bus initialized successfully.");

        // Subscribe to cmd_vel
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&MotorController::motorCallback, this, std::placeholders::_1));

        // Timer to send commands at fixed frequency (e.g., 10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorController::sendMotorCommand, this));

        RCLCPP_INFO(this->get_logger(), "Motor controller node started.");
    }

    ~MotorController() {
        if (file_ >= 0) {
            close(file_);
            RCLCPP_INFO(this->get_logger(), "I2C bus closed.");
        }
    }

private:
    void motorCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_twist_ = *msg;  // store latest message
    }

    void sendMotorCommand() {
        int16_t left_motor = static_cast<int16_t>(latest_twist_.linear.x * 180 - latest_twist_.angular.z * 250);
        int16_t right_motor = static_cast<int16_t>(latest_twist_.linear.x * 180 + latest_twist_.angular.z * 250);

        if (left_motor > 255) left_motor = 255;
        if (left_motor < -255) left_motor = -255;
        if (right_motor > 255) right_motor = 255;
        if (right_motor < -255) right_motor = -255;

        unsigned char data[6];
        data[0] = 0x00;                 // Dummy register
        data[1] = 0x55;                 // Streaming mode
        data[2] = (left_motor >> 8) & 0xFF;
        data[3] = left_motor & 0xFF;
        data[4] = (right_motor >> 8) & 0xFF;
        data[5] = right_motor & 0xFF;

        ssize_t written = write(file_, data, sizeof(data));
        if (written != sizeof(data)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to the I2C bus.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent [L: %d, R: %d]", left_motor, right_motor);
        }
    }

    int file_;
    geometry_msgs::msg::Twist latest_twist_;  // stores the last received twist message
    Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    init(argc, argv);
    auto node = std::make_shared<MotorController>();
    if (ok()) {
        spin(node);
    }
    shutdown();
    return 0;
}
