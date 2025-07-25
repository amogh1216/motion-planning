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
#include <queue>
#include <string>

#include "controls/diff_mp.cpp"
#include "controls/diff_pid_profiler.cpp"

using namespace std::chrono_literals;

class DiffDriveBrain : public rclcpp::Node
{
public:
    DiffDriveBrain() : Node("diff_drive_brain")
    {

        curr_pose_ = {0.0, 0.0, 0.0};
        goal_pose_ = {0.0, 0.0, 0.0};

        curr_time_ = this->now();
        command.header.frame_id = "base_link";

        curr_profile = nullptr;
        is_turning_ = false;

        prev_vel_ = 0;

        // Declare parameters (backup)
        this->declare_parameter<float>("p", 0.5);
        this->declare_parameter<float>("i", 0.0);
        this->declare_parameter<float>("d", 0.0);
        this->declare_parameter<float>("p_head", 0.2);
        this->declare_parameter<float>("i_head", 0.00);
        this->declare_parameter<float>("d_head", 0.0);
        this->declare_parameter<std::string>("control_mode", "motion_profile");

        pid = DiffPID(k_p, k_i, k_d, k_p_head_, k_i_head_, k_d_head_);
        pid_profiler = DiffPIDProfiler(k_p, k_i, k_d, k_p_head_, k_i_head_, k_d_head_, 
                5, 2, 4, 2);

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

        // Get parameters from yaml config
        getParams();
    }

private:

    void getParams()
    {
        this->get_parameter("p", k_p);
        this->get_parameter("i", k_i);
        this->get_parameter("d", k_d);
        this->get_parameter("p_head", k_p_head_);
        this->get_parameter("i_head", k_i_head_);
        this->get_parameter("d_head", k_d_head_);
        this->get_parameter("control_mode", control_mode);

        pid.updateGains(k_p, k_i, k_d, k_p_head_, k_i_head_, k_d_head_);
        pid_profiler.updateGains(k_p, k_i, k_d, k_p_head_, k_i_head_, k_d_head_);

        // Timer for publishing commands
        timer_ = this->create_wall_timer(
            50ms, std::bind(&DiffDriveBrain::motionProfile, this));

        RCLCPP_INFO(this->get_logger(),"control mode: %s", control_mode.c_str());

        if (control_mode == "pid_profiler")
        {
            timer_ = this->create_wall_timer(
                50ms, std::bind(&DiffDriveBrain::pidProfilerMove, this));
        }
        else if (control_mode == "pid")
        {
            timer_ = this->create_wall_timer(
                50ms, std::bind(&DiffDriveBrain::pidMove, this));
        }

        
    }

    void poseEstimateCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        tf2::Quaternion q(
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        curr_pose_ = {msg->pose.position.x, msg->pose.position.y, yaw};
        pid.updatePose(curr_pose_);
        pid_profiler.updatePose(curr_pose_);
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        tf2::Quaternion q(
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // goal_heading_ = yaw;
        // note the following is relative heading from curr point to goal point
        
        goal_pose_ = {msg->pose.position.x, msg->pose.position.y, 
            std::atan2(goal_pose_.y - curr_pose_.y, goal_pose_.x - curr_pose_.x)};

        pid.setTarget(goal_pose_.x, goal_pose_.y);
        pid_profiler.setTarget(goal_pose_.x, goal_pose_.y);
        state = MoveState::TURN;

        TrapezoidalMotionProfile turn_profile(M_PI, M_PI, true);
        turn_profile.initialize(goal_pose_.heading - curr_pose_.heading);
        motion_profiles_.push(turn_profile);

        TrapezoidalMotionProfile forward_profile(10, 5, false);
        motion_profiles_.push(forward_profile);


    }

    void motionProfile()
    {

        getParams();

        curr_time_ = this->now();
        command.header.stamp = curr_time_;
        command.header.frame_id = "base_link";

        // mp in progress
        if (curr_profile != nullptr)
        {
            profile_elapsed_time = curr_time_.seconds() - profile_start_time_.seconds();

            // move to next profile
            if (profile_elapsed_time > curr_profile->getDuration()) {
                curr_profile = nullptr;
                return;
            }

            command.twist.linear.x = 0;
            command.twist.linear.y = 0;
            command.twist.angular.z = 0;

            if (is_turning_) {
                command.twist.angular.z = curr_profile->getVelocity(profile_elapsed_time);
            }
            else {
                command.twist.linear.x = curr_profile->getVelocity(profile_elapsed_time);
            }

            RCLCPP_INFO(this->get_logger(), "MP time %0.3f/%0.3f, turning: %d", 
                profile_elapsed_time, curr_profile->getDuration(), is_turning_);


            RCLCPP_INFO(this->get_logger(), "Publishing linear velocity: x=%.5f, angular vel=%.5f",
                command.twist.linear.x, command.twist.angular.z);

            publisher_->publish(command);

        }
        // no mps 
        else if (motion_profiles_.empty()) {

            command.twist.linear.x = 0;
            command.twist.linear.y = 0;
            command.twist.angular.z = 0;
            RCLCPP_WARN(this->get_logger(), "No motion profiles available.");

            publisher_->publish(command);

            return;
        }
        // exists mp to pull
        else
        {
            curr_profile = &motion_profiles_.front();
            motion_profiles_.pop();
            if (curr_profile->getIsTurn()) {
                is_turning_ = true;
            }
            else {
                is_turning_ = false;
                curr_profile->initialize(pid.getGoalDistance());
            }
            profile_start_time_ = this->now();
        }

        RCLCPP_INFO(this->get_logger(), "Curr (%.2f, %.2f, %0.3f), Goal (%.2f, %.2f, %0.3f) dist: %0.5f",
            curr_pose_.x, curr_pose_.y, curr_pose_.heading, goal_pose_.x, goal_pose_.y, 
            pid.getGoalHeading(), pid.getGoalDistance() - pid.getDist());

    }

    void pidMove()
    {
        getParams();

        // Calculate time difference
        curr_time_ = this->now();
        double dt = curr_time_.seconds() - prev_time_.seconds();
        prev_time_ = curr_time_;
        
        if (dt <= 0) dt = 0.001;

        command.header.stamp = curr_time_;

        command.twist.linear.y = 0;
        command.twist.angular.z = 0;

        // defining state actions
        switch (state) {
            case MoveState::TURN:
                command.twist.angular.z = pid.computeAngularVelocity(dt);
                command.twist.linear.x = 0;
                break;
            case MoveState::STABILIZE_ANGLE:
                command.twist.angular.z = pid.computeAngularVelocity(dt);
                command.twist.linear.x = 0;
                break;
            case MoveState::MOVE:
                command.twist.angular.z = 0;
                command.twist.linear.x = pid.computeLinearVelocity(dt);
                break;
            case MoveState::STOP:
                command.twist.linear.x = 0;
                command.twist.angular.z = 0;
        }

        // defining state transitions

        // large angle error
        if (abs(pid.getHeadingErr()) > 0.05)
        {
            // only recalibrate when stabilizing
            if (state != MoveState::MOVE || state != MoveState::STOP)
            {
                state = MoveState::TURN;
                RCLCPP_INFO(this->get_logger(), "TURNING -- ERROR %0.3f", pid.getHeadingErr());
            }
        }
        // heading small enough but still turning 
        else if (state == MoveState::TURN)
        {
            stabilize_angle_time_ = curr_time_;
            state = MoveState::STABILIZE_ANGLE;
            
        }
        // small heading error but not enough to start moving
        else if (state == MoveState::STABILIZE_ANGLE)
        {
            double time_elapsed = curr_time_.seconds() - stabilize_angle_time_.seconds();
            if (abs(pid.getHeadingErr()) < 0.02)
            {
                command.twist.angular.z = 0;
                pid.resetGoalDistance();
                RCLCPP_WARN(this->get_logger(),"RESET GOAL DISTANCE BEFORE MOVING");

                if (time_elapsed > 2)
                {
                    state = MoveState::MOVE;
                }
            }
            RCLCPP_INFO(this->get_logger(), "STABILIZING -- ERROR %0.3f", pid.getHeadingErr());
        }
        // heading error minimized, can move directly forward
        else if (state == MoveState::MOVE)
        {
            if (pid.isMovingForward()) RCLCPP_INFO(this->get_logger(),"FORWARD");
            else if (command.twist.linear.x < 0) RCLCPP_INFO(this->get_logger(),"BACKWARD");
        }

        // if error to destination is small enough just stop
        if (abs(pid.getGoalDistance() - pid.getDist()) < 0.35) {
            state = MoveState::STOP;
        }

        RCLCPP_INFO(this->get_logger(), "P: %0.2f P_HEAD: %0.2f", k_p, k_p_head_);

        RCLCPP_INFO(this->get_logger(), "Curr (%.2f, %.2f, %0.3f), Goal (%.2f, %.2f, %0.3f) signed dist: %0.5f, raw dist: %0.5f",
            curr_pose_.x, curr_pose_.y, curr_pose_.heading, goal_pose_.x, goal_pose_.y,
            pid.getGoalHeading(), pid.getGoalDistance() - pid.getDist(), pid.getDistToGoal());

        RCLCPP_INFO(this->get_logger(), "Publishing linear velocity: (x=%.2f, y=%.2f), angular vel=%.2f",
            command.twist.linear.x, command.twist.linear.y, command.twist.angular.z);

        publisher_->publish(command);

    }

    void pidProfilerMove()
    {
        getParams();

        // Calculate time difference
        curr_time_ = this->now();
        double dt = curr_time_.seconds() - prev_time_.seconds();
        prev_time_ = curr_time_;
        
        if (dt <= 0) dt = 0.001;

        command.header.stamp = curr_time_;

        command.twist.linear.y = 0;
        command.twist.angular.z = 0;

        // defining state actions
        switch (state) {
            case MoveState::TURN:
                command.twist.angular.z = pid_profiler.computeAngularVelocity(dt, prev_ang_vel_);
                command.twist.linear.x = 0;
                break;
            case MoveState::STABILIZE_ANGLE:
                command.twist.angular.z = pid_profiler.computeAngularVelocity(dt, prev_ang_vel_);
                command.twist.linear.x = 0;
                break;
            case MoveState::MOVE:
                command.twist.angular.z = 0;
                command.twist.linear.x = pid_profiler.computeLinearVelocity(dt, prev_vel_);
                break;
            case MoveState::STOP:
                command.twist.linear.x = 0;
                command.twist.angular.z = 0;
        }

        // defining state transitions

        // large angle error
        if (abs(pid_profiler.getHeadingErr()) > 0.05)
        {
            // only recalibrate when stabilizing
            if (state != MoveState::MOVE || state != MoveState::STOP)
            {
                state = MoveState::TURN;
                RCLCPP_INFO(this->get_logger(), "TURNING -- ERROR %0.3f", pid_profiler.getHeadingErr());
            }
        }
        // heading small enough but still turning 
        else if (state == MoveState::TURN)
        {
            stabilize_angle_time_ = curr_time_;
            state = MoveState::STABILIZE_ANGLE;
            
        }
        // small heading error but not enough to start moving
        else if (state == MoveState::STABILIZE_ANGLE)
        {
            double time_elapsed = curr_time_.seconds() - stabilize_angle_time_.seconds();
            if (abs(pid_profiler.getHeadingErr()) < 0.02)
            {
                command.twist.angular.z = 0;
                pid_profiler.resetGoalDistance();
                RCLCPP_WARN(this->get_logger(),"RESET GOAL DISTANCE BEFORE MOVING");

                if (time_elapsed > 2)
                {
                    state = MoveState::MOVE;
                }
            }
            RCLCPP_INFO(this->get_logger(), "STABILIZING -- ERROR %0.3f", pid_profiler.getHeadingErr());
        }
        // heading error minimized, can move directly forward
        else if (state == MoveState::MOVE)
        {
            if (pid_profiler.isMovingForward()) RCLCPP_INFO(this->get_logger(),"FORWARD");
            else if (command.twist.linear.x < 0) RCLCPP_INFO(this->get_logger(),"BACKWARD");
        }

        // if error to destination is small enough just stop
        if (abs(pid_profiler.getGoalDistance() - pid_profiler.getDist()) < 0.35) {
            state = MoveState::STOP;
        }

        RCLCPP_INFO(this->get_logger(), "P: %0.2f P_HEAD: %0.2f", k_p, k_p_head_);

        RCLCPP_INFO(this->get_logger(), "Curr (%.2f, %.2f, %0.3f), Goal (%.2f, %.2f, %0.3f) signed dist: %0.5f, raw dist: %0.5f",
            curr_pose_.x, curr_pose_.y, curr_pose_.heading, goal_pose_.x, goal_pose_.y,
            pid_profiler.getGoalHeading(), pid_profiler.getGoalDistance() - pid_profiler.getDist(), 
            pid_profiler.getDistToGoal());

        RCLCPP_INFO(this->get_logger(), "Publishing linear velocity: (x=%.2f, y=%.2f), angular vel=%.2f",
            command.twist.linear.x, command.twist.linear.y, command.twist.angular.z);

        prev_vel_ = command.twist.linear.x;
        prev_ang_vel_ = command.twist.angular.z;

        publisher_->publish(command);

    }
    // Node components
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_estimate_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TwistStamped command;

    // localization + paths
    Pose2d curr_pose_;
    Pose2d goal_pose_;
    rclcpp::Time prev_time_;
    rclcpp::Time curr_time_;
    rclcpp::Time stabilize_angle_time_;

    MoveState state;

    // control mode
    std::string control_mode;

    // pid
    float k_p, k_i, k_d, k_p_head_, k_i_head_, k_d_head_;
    DiffPID pid;

    // pid profiler
    DiffPIDProfiler pid_profiler;
    float prev_vel_;
    float prev_ang_vel_;

    // motion profiler
    std::queue<TrapezoidalMotionProfile> motion_profiles_;
    rclcpp::Time profile_start_time_;
    float profile_elapsed_time;
    TrapezoidalMotionProfile* curr_profile;
    bool is_turning_;

};

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveBrain>());
    rclcpp::shutdown();
    return 0;
}
