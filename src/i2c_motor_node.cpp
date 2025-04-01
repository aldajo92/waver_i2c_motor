#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

extern "C" {
    #include <linux/i2c.h>
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#define I2C_BUS "/dev/i2c-1"
#define I2C_ADDR 0x11

class MotorController : public rclcpp::Node {
public:
    MotorController() : Node("motor_controller"), file_(-1) {
        // Initialize the I2C bus
        file_ = open(I2C_BUS, O_RDWR);
        if (file_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open the I2C bus.");
            rclcpp::shutdown();
            return;
        }

        if (ioctl(file_, I2C_SLAVE, I2C_ADDR) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to acquire bus access or talk to the I2C slave device.");
            close(file_);
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "I2C bus initialized successfully.");

        // Create the subscription
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&MotorController::motorCallback, this, std::placeholders::_1));

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
        // Map linear and angular velocity to motor values
        int16_t left_motor = static_cast<int16_t>(msg->linear.x * 1000 - msg->angular.z * 500);
        int16_t right_motor = static_cast<int16_t>(msg->linear.x * 1000 + msg->angular.z * 500);

        unsigned char data[4];
        data[0] = (left_motor >> 8) & 0xFF;
        data[1] = left_motor & 0xFF;
        data[2] = (right_motor >> 8) & 0xFF;
        data[3] = right_motor & 0xFF;

        if (i2c_smbus_write_i2c_block_data(file_, 0x00, 4, data) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to the I2C bus.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Command sent: [Left: %d, Right: %d]", left_motor, right_motor);
        }
    }

    int file_; // File descriptor for the I2C bus
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
