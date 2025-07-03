#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
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

        curr_x_ = 0.0; curr_y_ = 0.0; heading_ = 0.0;
        goal_x_ = 0.0; goal_y_ = 0.0; goal_heading_ = 0.0;

        // PID controller constants
        k_p = 1; k_i = 0.0; k_d = 0.0;
        k_p_head_=0.2; k_i_head_=0; k_d_head_=0;


        // Create publisher for velocity commands
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/rwd_diff_controller/cmd_vel", 10);

        pose_estimate_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/get_pose", 10, 
            std::bind(&DiffDriveBrain::poseEstimateCallback, this, std::placeholders::_1)
        );

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, 
            std::bind(&DiffDriveBrain::goalPoseCallback, this, std::placeholders::_1)
        );

        // Timer for publishing commands
        timer_ = this->create_wall_timer(
            50ms, std::bind(&DiffDriveBrain::publishControllerCommand, this));
    }

private:

    void poseEstimateCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        curr_x_ = msg->pose.position.x;
        curr_y_ = msg->pose.position.y;

        tf2::Quaternion q(
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        heading_ = yaw;
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Calculate time difference
        rclcpp::Time current_time = this->now();
        double dt = current_time.seconds() - prev_time_.seconds();
        prev_time_ = current_time;

        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;

        tf2::Quaternion q(
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        goal_heading_ = yaw;
    }

    void publishControllerCommand()
    {

        // Calculate time difference
        rclcpp::Time current_time = this->now();
        double dt = current_time.seconds() - prev_time_.seconds();
        prev_time_ = current_time;

        // step 1: pid turn to goal position
        float x_err_ = goal_x_ - curr_x_;
        float y_err_ = goal_y_ - curr_y_;

        tf2::Quaternion q(
                    x_err_,
                    y_err_,
                    0,
                    0
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        float heading_err_ = yaw - heading_;

        // step 2: pid linear movement to goal position

        // Maintain circular motion command
        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = this->now();
        command.header.frame_id = "base_link";
        command.twist.linear.x = 0.0;
        command.twist.angular.z = 0.1;

        command.twist.angular.z = k_p * heading_err_;
        command.twist.linear.x = k_p * x_err_;
        command.twist.linear.y = k_p * y_err_;

        RCLCPP_INFO(this->get_logger(),
                    "Curr (%.2f, %.2f), Goal (%.2f, %.2f)",
                    curr_x_, curr_y_, goal_x_, goal_y_);

        RCLCPP_INFO(this->get_logger(),
                    "Publishing linear velocity: (x=%.2f, y=%.2f), angular vel=%.2f",
                    k_p * x_err_, k_p * y_err_, k_p * heading_err_);

        publisher_->publish(command);
    }

    // Node components
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_estimate_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float curr_x_, curr_y_, heading_;
    float goal_x_, goal_y_, goal_heading_;
    rclcpp::Time prev_time_;

    float k_p, k_i, k_d, k_p_head_, k_i_head_, k_d_head_;

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveBrain>());
    rclcpp::shutdown();
    return 0;
}
