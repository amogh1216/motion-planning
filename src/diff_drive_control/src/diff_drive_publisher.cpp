#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>
using namespace std::chrono_literals;

class DiffDrivePublisher : public rclcpp::Node
{
    public:
        DiffDrivePublisher() : Node("diff_drive_publisher")
        {
            publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/rwd_diff_controller/cmd_vel", 10);

            timer_ = this->create_wall_timer(
                50ms, std::bind(&DiffDrivePublisher::publishCommand, this));
        }

    private:
        void publishCommand()
        {
            // circular motion
            geometry_msgs::msg::TwistStamped command;
            command.header.stamp = this->now();
            command.twist.linear.x=0.3;
            command.twist.angular.z=0.5;
            publisher_->publish(command);
        }

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDrivePublisher>());
    rclcpp::shutdown();
    return 0;
}