#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

/**
 * This node is used for localization of the robot. Currently there are two 
 * separate localization techniques. The estimated robot position is published.
 * --- /rwd_diff_controller/odom: derived from differential drive kinematics based on 
 * (linear, angular) velocity commands
 * --- /imu: from imu sensor on the robot. I use a method to estimate current position 
 * based on linear acceleration
 * 
 * I also have the absolute position of the robot based on gazebo that I receive f
 */

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode() : Node("localization_node")
    {

        // Initialize position and velocity
        reset();

        // Publisher for the current pose
        pose_pub_ = this->
            create_publisher<geometry_msgs::msg::PoseStamped>("/get_pose", 10);

        // Create IMU subscriber
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, 
            std::bind(&LocalizationNode::imuCallback, this, std::placeholders::_1));

        // Create Odom subscriber
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rwd_diff_controller/odom", 10,
            std::bind(&LocalizationNode::odomCallback, this, std::placeholders::_1));

        // Subscriber to TF
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/world/my_world/pose/info", 10,
            std::bind(&LocalizationNode::tfCallback, this, std::placeholders::_1));
    }

private:

    void reset()
    {
        imu_position_x_ = 0.0;
        imu_position_y_ = 0.0;
        imu_heading_ = 0.0;
        velocity_x_ = 0.0;
        velocity_y_ = 0.0;
        heading_ = 0.0;
        last_time_ = this->now();
    }

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& transform : msg->transforms) {
            if (transform.child_frame_id == "rwd_bot") {
                // Extract position and orientation
                
                pose_msg.header = transform.header;
                pose_msg.header.stamp = this->now();
                pose_msg.pose.position.x = transform.transform.translation.x;
                pose_msg.pose.position.y = transform.transform.translation.y;
                pose_msg.pose.position.z = transform.transform.translation.z;
                pose_msg.pose.orientation = transform.transform.rotation;

                // Optionally extract heading (yaw)
                tf2::Quaternion q(
                    pose_msg.pose.orientation.x,
                    pose_msg.pose.orientation.y,
                    pose_msg.pose.orientation.z,
                    pose_msg.pose.orientation.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                heading_ = yaw;
                // Publish the pose
                pose_pub_->publish(pose_msg);
                break;
            }
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Calculate time difference
        rclcpp::Time current_time = this->now();
        double dt = current_time.seconds() - last_time_.seconds(); //(current_time - last_time_).seconds();
        last_time_ = current_time;

        // Extract linear acceleration
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;

        // RCLCPP_INFO(this->get_logger(), "IMU dt: (%.8f) accel: (%.5f, %.5f)", dt, ax, ay);

        // Convert acceleration to world frame using current heading
        double ax_world = ax * std::cos(imu_heading_) - ay * std::sin(imu_heading_);
        double ay_world = ax * std::sin(imu_heading_) + ay * std::cos(imu_heading_);

        // Integrate acceleration to update velocity
        velocity_x_ += ax_world * dt;
        velocity_y_ += ay_world * dt;

        // Integrate velocity to update position
        imu_position_x_ += velocity_x_ * dt;
        imu_position_y_ += velocity_y_ * dt;

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
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position
        double odom_x = msg->pose.pose.position.x;
        double odom_y = msg->pose.pose.position.y;
        double odom_z = msg->pose.pose.position.z;
        
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
        RCLCPP_INFO(this->get_logger(), "ODOM Position: (%.2f, %.2f, %.2f), Heading: %.2f rad", 
                    odom_x, odom_y, odom_z, yaw);

        RCLCPP_INFO(this->get_logger(), "IMU Position: (%.2f, %.2f), Heading: %.2f rad", 
                    imu_position_x_, imu_position_y_, imu_heading_);

        RCLCPP_INFO(this->get_logger(),
                    "Publishing pose: x=%.2f, y=%.2f, z=%.2f, heading(yaw)=%.2f rad",
                    pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, yaw);
    }

    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

    // State variables for IMU reconstruction
    rclcpp::Time last_time_;
    double imu_position_x_;
    double imu_position_y_;
    double imu_heading_;
    double velocity_x_;
    double velocity_y_;

    // true pose
    geometry_msgs::msg::PoseStamped pose_msg;
    double heading_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
