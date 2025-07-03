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
        prev_x_ = 0.0; prev_y_ = 0.0; prev_heading_ = 0.0;
        x_err_integral_ = 0.0; y_err_integral_ = 0.0; heading_err_integral_ = 0.0;
        goal_x_ = 0.0; goal_y_ = 0.0; goal_heading_ = 0.0;

        // Declare parameters
        this->declare_parameter<float>("p", 0.7);
        this->declare_parameter<float>("i", 0.0);
        this->declare_parameter<float>("d", 0.0);
        this->declare_parameter<float>("p_head", 0.2);
        this->declare_parameter<float>("i_head", 0.0);
        this->declare_parameter<float>("d_head", 0.0);

        // Get parameters
        this->get_parameter("p", k_p);
        this->get_parameter("i", k_i);
        this->get_parameter("d", k_d);
        this->get_parameter("p_head", k_p_head_);
        this->get_parameter("i_head", k_i_head_);
        this->get_parameter("d_head", k_d_head_);


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
        prev_x_ = curr_x_;
        prev_y_ = curr_y_;

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

        prev_heading_ = heading_;
        heading_ = yaw;
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

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

        if (dt <= 0.0) dt = 0.001;

        float x_err_ = goal_x_ - curr_x_;
        float y_err_ = goal_y_ - curr_y_;

        double desired_heading = std::atan2(y_err_, x_err_);
        double heading_err_ = desired_heading - heading_;
        
        // if (heading_err_ > 3.14) heading_err_ -= (2.0 * 3.14);
        // else if (heading_err_ < -3.14) heading_err_ += (2.0 * 3.14);

        x_err_integral_ += x_err_ * dt;
        y_err_integral_ += y_err_ * dt;
        heading_err_integral_ += heading_err_ * dt;


        // Maintain circular motion command
        geometry_msgs::msg::TwistStamped command;
        command.header.stamp = this->now();
        command.header.frame_id = "base_link";

        // PID controller
        if (std::isnan(command.twist.linear.x)) command.twist.linear.x = 0.0;
        else command.twist.linear.x = (k_p * x_err_)  + (k_i * x_err_integral_) + (k_d * (curr_x_ - prev_x_) / dt);

        if (std::isnan(command.twist.linear.y)) command.twist.linear.y = 0.0;
        else command.twist.linear.y = (k_p * y_err_) + (k_i * y_err_integral_) + (k_d * (curr_y_ - prev_y_) / dt);

        if (std::isnan(command.twist.angular.z)) command.twist.angular.z = 0.0;
        else command.twist.angular.z = (k_p_head_ * heading_err_) + (k_i_head_ * heading_err_integral_) + (k_d_head_ * (heading_ - prev_heading_) / dt);

        RCLCPP_INFO(this->get_logger(),
                    "Curr (%.2f, %.2f, %0.3f), Goal (%.2f, %.2f, %0.3f)",
                    curr_x_, curr_y_, heading_, goal_x_, goal_y_, goal_heading_);

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

    double curr_x_, curr_y_, heading_;
    double prev_x_, prev_y_, prev_heading_;
    double x_err_integral_, y_err_integral_, heading_err_integral_;
    double goal_x_, goal_y_, goal_heading_;
    rclcpp::Time prev_time_;

    float k_p, k_i, k_d, k_p_head_, k_i_head_, k_d_head_;

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveBrain>());
    rclcpp::shutdown();
    return 0;
}
