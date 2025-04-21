#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

using namespace std::chrono_literals;

class I2CVelocityNode : public rclcpp::Node {
public:
    I2CVelocityNode() : Node("i2c_velocity_node"), left_wheel_angle_(0.0), right_wheel_angle_(0.0) {
        declare_parameter("i2c_device", "/dev/i2c-1");
        declare_parameter("i2c_address", 0x11);
        declare_parameter("wheel_radius", 0.08); // in meters
        declare_parameter("wheel_base", 0.128);  // in meters
        declare_parameter("polling_rate_hz", 5.0);
        declare_parameter("vx_scaling_factor", 1.7); // New parameter for scaling factor
        declare_parameter("vw_scaling_factor", 2.4); // New parameter for scaling factor
        declare_parameter("left_wheel_joint_name", "wheel_front_left_joint");
        declare_parameter("right_wheel_joint_name", "wheel_front_right_joint");
        declare_parameter("left_wheel_back_joint_name", "wheel_back_left_joint");
        declare_parameter("right_wheel_back_joint_name", "wheel_back_right_joint");

        get_parameter("i2c_device", i2c_device_);
        get_parameter("i2c_address", i2c_address_);
        get_parameter("wheel_radius", wheel_radius);
        get_parameter("wheel_base", wheel_base);
        double freq;
        get_parameter("polling_rate_hz", freq);
        get_parameter("vx_scaling_factor", vx_scaling_factor_);
        get_parameter("vw_scaling_factor", vw_scaling_factor_);

        get_parameter("left_wheel_joint_name", left_wheel_joint_name);
        get_parameter("right_wheel_joint_name", right_wheel_joint_name);
        get_parameter("left_wheel_back_joint_name", left_wheel_back_joint_name);
        get_parameter("right_wheel_back_joint_name", right_wheel_back_joint_name);

        vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/vel_from_i2c", 10);
        vel_string_pub_ = create_publisher<std_msgs::msg::String>("/vel_from_i2c_string", 10);
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        file_ = open(i2c_device_.c_str(), O_RDWR);
        if (file_ < 0 || ioctl(file_, I2C_SLAVE, i2c_address_) < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to open or connect to I2C device.");
            rclcpp::shutdown();
            return;
        }

        last_time_ = now();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / freq)),
            std::bind(&I2CVelocityNode::readVelocity, this));
    }

    ~I2CVelocityNode() {
        if (file_ >= 0)
            close(file_);
    }

private:
    void readVelocity() {
        auto current_time = now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        get_parameter("vx_scaling_factor", vx_scaling_factor_);
        get_parameter("vw_scaling_factor", vw_scaling_factor_);

        uint8_t buffer[4];
        if (read(file_, buffer, 4) != 4) {
            RCLCPP_WARN(get_logger(), "Failed to read from I2C");
            return;
        }

        int16_t rpm_l = (buffer[0] << 8) | buffer[1];
        int16_t rpm_r = (buffer[2] << 8) | buffer[3];

        double rpmL = rpm_l / 100.0;
        double rpmR = rpm_r / 100.0;

        double v_l = (rpmL * 2 * M_PI * wheel_radius) / 60.0;
        double v_r = (rpmR * 2 * M_PI * wheel_radius) / 60.0;

        double v = (v_r + v_l) / (2.0 * vx_scaling_factor_); // vx_scaling_factor_ is a fix factor
        double w = (v_r - v_l) / (2.0 * vw_scaling_factor_ * wheel_base); // vw_scaling_factor_ is a fix factor

        // Update wheel angles using raw RPM values
        left_wheel_angle_ += (rpmL * 2 * M_PI / 60.0) * dt; // rad/s * s = rad
        right_wheel_angle_ += (rpmR * 2 * M_PI / 60.0) * dt;

        // Publish velocity
        geometry_msgs::msg::Twist twist;
        twist.linear.x = v;
        twist.angular.z = w;
        vel_pub_->publish(twist);

        std_msgs::msg::String msg;
        char str_buffer[64];
        snprintf(str_buffer, sizeof(str_buffer), "L:%.1f,R:%.1f", v_l, v_r);
        msg.data = std::string(str_buffer);
        vel_string_pub_->publish(msg);

        // Publish joint states
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = current_time;
        joint_state.name = {left_wheel_joint_name, right_wheel_joint_name, 
                            left_wheel_back_joint_name, right_wheel_back_joint_name};
        joint_state.position = {left_wheel_angle_, right_wheel_angle_,
                                left_wheel_angle_, right_wheel_angle_};
        joint_pub_->publish(joint_state);
    }

    std::string i2c_device_;
    int i2c_address_;
    double wheel_radius, wheel_base;
    double vx_scaling_factor_, vw_scaling_factor_;
    int file_;
    double left_wheel_angle_, right_wheel_angle_;
    std::string left_wheel_joint_name, right_wheel_joint_name, 
        left_wheel_back_joint_name, right_wheel_back_joint_name;
    rclcpp::Time last_time_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vel_string_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<I2CVelocityNode>());
    rclcpp::shutdown();
    return 0;
}
