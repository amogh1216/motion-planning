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
 * True position received from Gazebo transform.
 */

bool GAZEBO_FDBACK = true;

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode() : Node("localization_node")
    {

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

        this->declare_parameter<std::string>("mode", "truth");

        // Position format: {x, y, z, heading}
        imu_pose_ = {0.0, 0.0, 0.0, 0.0};
        odom_pose_ = {0.0, 0.0, 0.0, 0.0};
        truth_pose_ = {0.0, 0.0, 0.0, 0.0};

        // velocity format: {x, y}
        imu_vel_ = {0.0, 0.0};

        last_time_ = this->now();

        // loads localizer mode: 'truth' (gazebo) or 'odom' (diffdrivecontroller)
        log_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),   // change interval as needed
            std::bind(&LocalizationNode::callback, this)
        );
    }

private:

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& transform : msg->transforms) {
            if (transform.child_frame_id == "rwd_bot" && GAZEBO_FDBACK) {
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

                // Save pose
                truth_pose_ = {pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, yaw};

                // Publish the pose
                if (localizer == "truth") {
                    pose_pub_->publish(pose_msg);
                }
                break;
            }
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Calculate time difference
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Extract linear acceleration
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;

        // RCLCPP_INFO(this->get_logger(), "IMU dt: (%.8f) accel: (%.5f, %.5f)", dt, ax, ay);

        // Extract heading from orientation quaternion
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Convert acceleration to world frame using current heading
        double ax_world = ax * std::cos(yaw) - ay * std::sin(yaw);
        double ay_world = ax * std::sin(yaw) + ay * std::cos(yaw);

        // Integrate acceleration to update velocity
        imu_vel_[0] += ax_world * dt;
        imu_vel_[1] += ay_world * dt;

        // Integrate velocity to update position
        double delta_x = imu_vel_[0] * dt;
        double delta_y = imu_vel_[1] * dt;

        imu_pose_ = {imu_pose_[0] + delta_x, imu_pose_[1] + delta_y, 0.0, yaw};
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        
        // Extract orientation and convert to heading
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Extract pose
        odom_pose_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw};

        // Extract position and orientation        
        if (localizer == "odom") {
            pose_msg.header.stamp = this->now();
            pose_msg.pose.position.x = odom_pose_[0];
            pose_msg.pose.position.y = odom_pose_[1];
            pose_msg.pose.position.z = odom_pose_[2];
            pose_msg.pose.orientation = msg->pose.pose.orientation;

            // Publish the pose
            pose_pub_->publish(pose_msg);
        }
    }

    void logPoseData(const std::string &source, const std::array<double,4> &p)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Mode [%s] | [%s] Position: (%.2f, %.2f, %.2f), Heading: %.2f rad",
                    localizer.c_str(), source.c_str(), p[0], p[1], p[2], p[3]);
    }

    void callback()
    {
        logPoseData("ODOM", odom_pose_);
        logPoseData("IMU", imu_pose_);
        logPoseData("TRUTH", truth_pose_);
        this->get_parameter("mode", localizer);

        if (localizer != "odom" and localizer != "truth")
        {
            RCLCPP_ERROR(this->get_logger(), "Localization mode set to %s. Should be set to either \"odom\" or \"truth.\" ", localizer.c_str());
        }
        
    }

    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

    rclcpp::Time last_time_;
    rclcpp::TimerBase::SharedPtr log_timer_;

    std::array<double, 4> imu_pose_;
    std::array<double, 4> odom_pose_;
    std::array<double, 4> truth_pose_;
    std::array<double, 2> imu_vel_;

    geometry_msgs::msg::PoseStamped pose_msg;
    std::string localizer;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
