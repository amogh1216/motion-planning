#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class CreateMapNode : public rclcpp::Node
{
public:
    CreateMapNode() : Node("create_map_node"), create_map_active_(false)
    {
        // Subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/get_pose_est", 10, std::bind(&CreateMapNode::poseCallback, this, std::placeholders::_1));

        create_map_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/create_map", 10, std::bind(&CreateMapNode::createMapCallback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/rwd_diff_controller/cmd_vel", 10, std::bind(&CreateMapNode::cmdVelCallback, this, std::placeholders::_1));

        point_cloud_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/point_cloud", 10, std::bind(&CreateMapNode::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for robot movement
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/rwd_diff_controller/cmd_vel", 10);

        // Timer to periodically call the moveRobot method
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&CreateMapNode::moveRobot, this));
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        //RCLCPP_INFO(this->get_logger(), "Received pose: x=%.2f, y=%.2f", msg->pose.position.x, msg->pose.position.y);
    }

    void pointCloudCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        point_cloud_.clear(); // Clear the previous point cloud data

        auto point_cloud_unshaped = msg->data;

        // Populate the point cloud as a vector of 2D arrays from 1D array
        for (size_t i = 0; i < point_cloud_unshaped.size(); i += 2)
        {
            if (i + 1 < point_cloud_unshaped.size()) // Ensure there are pairs of x, y values
            {
                point_cloud_.push_back({point_cloud_unshaped[i], point_cloud_unshaped[i + 1]});
            }
        }

        // Print the processed point_cloud_
        std::ostringstream oss;
        oss << "Processed point_cloud_: [";
        for (size_t i = 0; i < point_cloud_.size(); ++i) {
            oss << "(" << point_cloud_[i][0] << ", " << point_cloud_[i][1] << ")";
            if (i != point_cloud_.size() - 1) oss << ", ";
        }
        oss << "]";
        //RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    }

    void createMapCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        create_map_active_ = msg->data;
        if (create_map_active_)
        {
            RCLCPP_INFO(this->get_logger(), "Create map activated. Robot will start moving.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Create map deactivated. Robot will stop moving.");
        }
    }

    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        current_cmd_vel_ = *msg;
    }

    void moveRobot()
    {
        if (create_map_active_)
        {
            bool obstacle_in_front = false;

            // Use robot's orientation (yaw) from current_pose_.pose.orientation.z (original definition)
            double robot_angle = current_pose_.pose.position.z;

            // Check if any point in the point cloud is within the threshold distance in front of the robot
            for (const auto &point : point_cloud_)
            {
                double dx = point[0];
                double dy = point[1];
                double distance = std::sqrt(dx * dx + dy * dy);

                // Angle between robot's heading and the point (relative to robot, so just atan2)
                double angle_to_point = std::atan2(dy, dx);
                double angle_diff = std::fabs(angle_to_point - robot_angle);

                // problem: angle drift massive
                RCLCPP_INFO(this->get_logger(), "angle diff: %.2f", angle_diff);

                if (distance < thresh)
                {
                    obstacle_in_front = true;
                    break;
                }
            }

            auto cmd = geometry_msgs::msg::TwistStamped();
            cmd.header.stamp = this->now();

            // Smooth velocity adjustment
            double max_linear_velocity = 0.5; // Maximum forward velocity
            double max_angular_velocity = 0.5; // Maximum turning velocity
            double acceleration = 0.05; // Linear acceleration step
            double angular_acceleration = 0.05; // Angular acceleration step

            if (obstacle_in_front)
            {
                // Gradually stop linear velocity and increase angular velocity
                current_linear_velocity_ = std::max(0.0, current_linear_velocity_ - acceleration);
                current_angular_velocity_ = std::min(max_angular_velocity, current_angular_velocity_ + angular_acceleration);
            }
            else
            {
                // Gradually increase linear velocity and stop angular velocity
                current_linear_velocity_ = std::min(max_linear_velocity, current_linear_velocity_ + acceleration);
                current_angular_velocity_ = std::max(0.0, current_angular_velocity_ - angular_acceleration);
            }

            cmd.twist.linear.x = current_linear_velocity_;
            cmd.twist.angular.z = current_angular_velocity_;
            cmd_vel_pub_->publish(cmd);
        }
        else
        {
            // Gradually stop the robot
            auto cmd = geometry_msgs::msg::TwistStamped();
            cmd.header.stamp = this->now();

            double deceleration = 0.05; // Deceleration step
            current_linear_velocity_ = std::max(0.0, current_linear_velocity_ - deceleration);
            current_angular_velocity_ = std::max(0.0, current_angular_velocity_ - deceleration);

            cmd.twist.linear.x = current_linear_velocity_;
            cmd.twist.angular.z = current_angular_velocity_;
            cmd_vel_pub_->publish(cmd);

            if (current_linear_velocity_ == 0.0 && current_angular_velocity_ == 0.0)
            {
                RCLCPP_INFO(this->get_logger(), "STOPPED");
            }
        }
    }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr create_map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr point_cloud_sub_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    bool create_map_active_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::TwistStamped current_cmd_vel_;
    std::vector<std::array<double, 2>> point_cloud_;
    double thresh = 5;

    // State variables for smooth velocity control
    double current_linear_velocity_ = 0.0;
    double current_angular_velocity_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CreateMapNode>());
    rclcpp::shutdown();
    return 0;
}
