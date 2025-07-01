#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class DiffDriveBrain : public rclcpp::Node
{
public:
    DiffDriveBrain() : Node("diff_drive_brain")
    {
        // Initialize position and velocity
        imu_position_x_ = 0.0;
        imu_position_y_ = 0.0;
        imu_heading_ = 0.0;
        velocity_x_ = 0.0;
        velocity_y_ = 0.0;
        last_time_ = this->now();

        // Create publisher for velocity commands
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/rwd_diff_controller/cmd_vel", 10);

        // Create IMU subscriber
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, 
            std::bind(&DiffDriveBrain::imuCallback, this, std::placeholders::_1));

        // Create Odom subscriber
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rwd_diff_controller/odom", 10,
            std::bind(&DiffDriveBrain::odomCallback, this, std::placeholders::_1));

        // Timer for publishing commands
        timer_ = this->create_wall_timer(
            50ms, std::bind(&DiffDriveBrain::publishCommand, this));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Calculate time difference
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Extract linear acceleration
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;

        // Convert acceleration to world frame using current heading
        double ax_world = ax * std::cos(imu_heading_) - ay * std::sin(imu_heading_);
        double ay_world = ax * std::sin(imu_heading_) + ay * std::cos(imu_heading_);

        // Integrate acceleration to update velocity
        velocity_x_ += ax_world * dt;
        velocity_y_ += ay_world * dt;

        // Integrate velocity to update position
        imu_position_x_ += velocity_x_ * dt;
        imu_position_y_ += velocity_y_ * dt;

        // imu_position_y seems to be negative
        // imu_position_y_ *= -1;

        // Extract heading from orientation quaternion
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        imu_heading_ = yaw;

        // Log IMU-based position and heading
        // RCLCPP_INFO(this->get_logger(), "IMU Position: (%.2f, %.2f), Heading: %.2f rad", 
        //             imu_position_x_, imu_position_y_, imu_heading_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position
        double odom_x = msg->pose.pose.position.x;
        double odom_y = msg->pose.pose.position.y;
        
        // Extract orientation and convert to heading
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // Log odometry-based position and heading
        RCLCPP_INFO(this->get_logger(), "ODOM Position: (%.2f, %.2f), Heading: %.2f rad", 
                    odom_x, odom_y, yaw);
        
        // Compare with IMU reconstruction
        RCLCPP_INFO(this->get_logger(), "Position Error: X: %.2fm, Y: %.2fm | Heading Error: %.2f rad",
                    imu_position_x_ - odom_x,
                    imu_position_y_ - odom_y,
                    imu_heading_ - yaw);

        RCLCPP_INFO(this->get_logger(), "IMU Position: (%.2f, %.2f), Heading: %.2f rad", 
                    imu_position_x_, imu_position_y_, imu_heading_);
    }

    void publishCommand()
    {
        // Maintain circular motion command
        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = this->now();
        command.header.frame_id = "base_link";
        command.twist.linear.x = 0.05;
        command.twist.angular.z = 0;
        publisher_->publish(command);
    }

    // Node components
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables for IMU reconstruction
    rclcpp::Time last_time_;
    double imu_position_x_;
    double imu_position_y_;
    double imu_heading_;
    double velocity_x_;
    double velocity_y_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveBrain>());
    rclcpp::shutdown();
    return 0;
}
