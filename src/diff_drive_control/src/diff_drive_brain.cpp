#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <queue>
#include <string>
#include <chrono>
#include <cmath>
#include <memory>

#include "controls/diff_mp.cpp"
#include "controls/diff_pid_profiler.cpp"

using namespace std::chrono_literals;

class DiffDriveController {
public:
    virtual ~DiffDriveController() = default;
    virtual void updatePose(const Pose2d &pose) = 0;
    virtual void setTarget(const Pose2d &target) = 0;
    virtual void update(rclcpp::Time now,
                        geometry_msgs::msg::TwistStamped &cmd) = 0;
    virtual void updateParams(float p, float i, float d,
                               float p_head, float i_head, float d_head) = 0;

    virtual Pose2d getCurrentPose() const = 0;
    virtual Pose2d getGoalPose() const = 0;
    virtual double getDistance() const = 0;
    virtual std::string getState() const = 0;
    virtual double getVel() const = 0;
};

class PIDController : public DiffDriveController {
public:
    PIDController(float p, float i, float d,
                  float p_head, float i_head, float d_head)
      : pid_(p, i, d, p_head, i_head, d_head)
    {}

    void updatePose(const Pose2d &pose) override {
        curr_pose_ = pose;
        pid_.updatePose(pose);
    }

    void setTarget(const Pose2d &target) override {
        goal_pose_ = target;
        pid_.setTarget(goal_pose_.x, goal_pose_.y);
        state_ = MoveState::TURN;
    }

    void updateParams(float p, float i, float d,
                      float p_head, float i_head, float d_head) override {
        pid_.updateGains(p, i, d, p_head, i_head, d_head);
    }

    void update(rclcpp::Time now, geometry_msgs::msg::TwistStamped &cmd) override {
        double dt = (prev_time_.nanoseconds() == 0) ? 0.01 :
                    (now - prev_time_).seconds();
        prev_time_ = now;

        cmd.header.stamp = now;
        cmd.twist.linear.y = 0;
        cmd.twist.angular.z = 0;

        switch (state_) {
            case MoveState::TURN:
                cmd.twist.angular.z = pid_.computeAngularVelocity(dt);
                vel_ = cmd.twist.angular.z;
                break;
            case MoveState::STABILIZE_ANGLE:
                cmd.twist.angular.z = pid_.computeAngularVelocity(dt);
                vel_ = cmd.twist.angular.z;
                break;
            case MoveState::MOVE:
                cmd.twist.linear.x = pid_.computeLinearVelocity(dt);
                vel_ = cmd.twist.linear.x;
                break;
            case MoveState::STOP:
                vel_ = cmd.twist.linear.x;
                break;
        }

        // State transitions
        if (std::abs(pid_.getHeadingErr()) > 0.05) {
            if (state_ != MoveState::MOVE && state_ != MoveState::STOP)
                state_ = MoveState::TURN;
        } else if (state_ == MoveState::TURN) {
            stabilize_angle_time_ = now;
            state_ = MoveState::STABILIZE_ANGLE;
        } else if (state_ == MoveState::STABILIZE_ANGLE) {
            double elapsed = (now - stabilize_angle_time_).seconds();
            if (std::abs(pid_.getHeadingErr()) < 0.02) {
                pid_.resetGoalDistance();
                if (elapsed > 2.0)
                    state_ = MoveState::MOVE;
            }
        }
        if (std::abs(pid_.getGoalDistance() - pid_.getDist()) < 0.35) {
            state_ = MoveState::STOP;
        }
    }

    Pose2d getCurrentPose() const override { return curr_pose_; }
    Pose2d getGoalPose() const override { return goal_pose_; }
    double getDistance() const override { return std::abs(pid_.getGoalDistance() - pid_.getDist()); }
    std::string getState() const override { 
        if (state_ == MoveState::TURN) return "Turning";
        else if (state_ == MoveState::MOVE) return "Moving";
        else if (state_ == MoveState::STABILIZE_ANGLE) return "Stabilizing";
        else if (state_ == MoveState::STOP) return "Stopping";
        return "";
    }
    // angular or linear
    double getVel() const override { return vel_;}

private:
    DiffPID pid_;
    Pose2d curr_pose_{}, goal_pose_{};
    MoveState state_{MoveState::STOP};
    rclcpp::Time prev_time_;
    rclcpp::Time stabilize_angle_time_;
    double vel_;
};

class PIDProfilerController : public DiffDriveController {
public:
    PIDProfilerController(float p, float i, float d,
                          float p_head, float i_head, float d_head)
      : pid_profiler_(p, i, d, p_head, i_head, d_head, 5, 2, 4, 2)
    {}

    void updatePose(const Pose2d &pose) override {
        curr_pose_ = pose;
        pid_profiler_.updatePose(pose);
    }

    void setTarget(const Pose2d &target) override {
        goal_pose_ = target;
        pid_profiler_.setTarget(goal_pose_.x, goal_pose_.y);
        state_ = MoveState::TURN;
    }

    void updateParams(float p, float i, float d,
                      float p_head, float i_head, float d_head) override {
        pid_profiler_.updateGains(p, i, d, p_head, i_head, d_head);
    }

    void update(rclcpp::Time now, geometry_msgs::msg::TwistStamped &cmd) override {
        double dt = (prev_time_.nanoseconds() == 0) ? 0.01 :
                    (now - prev_time_).seconds();
        prev_time_ = now;

        cmd.header.stamp = now;
        cmd.twist.linear.y = 0;
        cmd.twist.angular.z = 0;

        switch (state_) {
            case MoveState::TURN:
                cmd.twist.angular.z = pid_profiler_.computeAngularVelocity(dt, prev_ang_vel_);
                vel_ = cmd.twist.angular.z;
                break;
            case MoveState::STABILIZE_ANGLE:
                cmd.twist.angular.z = pid_profiler_.computeAngularVelocity(dt, prev_ang_vel_);
                vel_ = cmd.twist.angular.z;
                break;
            case MoveState::MOVE:
                cmd.twist.linear.x = pid_profiler_.computeLinearVelocity(dt, prev_vel_);
                vel_ = cmd.twist.linear.x;
                break;
            case MoveState::STOP:
                vel_ = cmd.twist.linear.x;
                break;
        }
        prev_vel_ = cmd.twist.linear.x;
        prev_ang_vel_ = cmd.twist.angular.z;

        // State transitions
        if (std::abs(pid_profiler_.getHeadingErr()) > 0.05) {
            if (state_ != MoveState::MOVE && state_ != MoveState::STOP)
                state_ = MoveState::TURN;
        } else if (state_ == MoveState::TURN) {
            stabilize_angle_time_ = now;
            state_ = MoveState::STABILIZE_ANGLE;
        } else if (state_ == MoveState::STABILIZE_ANGLE) {
            double elapsed = (now - stabilize_angle_time_).seconds();
            if (std::abs(pid_profiler_.getHeadingErr()) < 0.02) {
                pid_profiler_.resetGoalDistance();
                if (elapsed > 2.0)
                    state_ = MoveState::MOVE;
            }
        }
        if (std::abs(pid_profiler_.getGoalDistance() - pid_profiler_.getDist()) < 0.35) {
            state_ = MoveState::STOP;
        }
    }

    Pose2d getCurrentPose() const override { return curr_pose_; }
    Pose2d getGoalPose() const override { return goal_pose_; }
    double getDistance() const override { return std::abs(pid_profiler_.getGoalDistance() - pid_profiler_.getDist()); }
    std::string getState() const override {
        if (state_ == MoveState::TURN) return "Turning";
        else if (state_ == MoveState::MOVE) return "Moving";
        else if (state_ == MoveState::STABILIZE_ANGLE) return "Stabilizing";
        else if (state_ == MoveState::STOP) return "Stopping";
        return "";
    }
    double getVel() const override {return vel_;}

private:
    DiffPIDProfiler pid_profiler_;
    Pose2d curr_pose_{}, goal_pose_{};
    MoveState state_{MoveState::STOP};
    rclcpp::Time prev_time_;
    rclcpp::Time stabilize_angle_time_;
    float prev_vel_{0}, prev_ang_vel_{0};
    double vel_;
};

class MotionProfileController : public DiffDriveController {
public:
    MotionProfileController() = default;

    void updatePose(const Pose2d &pose) override {
        curr_pose_ = pose;
    }

    void setTarget(const Pose2d &target) override {
        goal_pose_ = target;
        // Create turn + forward profiles
        TrapezoidalMotionProfile turn_profile(M_PI, M_PI, true);
        turn_profile.initialize(goal_pose_.heading - curr_pose_.heading);
        profiles_.push(turn_profile);

        TrapezoidalMotionProfile forward_profile(10, 5, false);
        profiles_.push(forward_profile);
    }

    void update(rclcpp::Time now, geometry_msgs::msg::TwistStamped &cmd) override {
        cmd.header.stamp = now;
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = cmd.twist.linear.y = cmd.twist.angular.z = 0;

        if (curr_profile_ != nullptr) {
            double elapsed = (now - start_time_).seconds();
            if (elapsed > curr_profile_->getDuration()) {
                curr_profile_ = nullptr;
                return;
            }
            if (curr_profile_->getIsTurn()) {
                cmd.twist.angular.z = curr_profile_->getVelocity(elapsed);
                vel_ = cmd.twist.angular.z;
            } else {
                cmd.twist.linear.x = curr_profile_->getVelocity(elapsed);
                vel_ = cmd.twist.linear.x;
            }
            return;
        }

        if (profiles_.empty())
            return;

        curr_profile_ = &profiles_.front();
        profiles_.pop();
        if (!curr_profile_->getIsTurn())
            curr_profile_->initialize(distanceToGoal());
        start_time_ = now;
    }

    void updateParams(float, float, float, float, float, float) override 
    {
        // no-op
    }


    Pose2d getCurrentPose() const override { return curr_pose_; }
    Pose2d getGoalPose() const override { return goal_pose_; }
    double getDistance() const override { return distanceToGoal(); }
    std::string getState() const override {return "";}
    double getVel() const override {return vel_;}


private:
    double distanceToGoal() const {
        return std::hypot(goal_pose_.x - curr_pose_.x, goal_pose_.y - curr_pose_.y);
    }

    Pose2d curr_pose_{}, goal_pose_{};
    std::queue<TrapezoidalMotionProfile> profiles_;
    TrapezoidalMotionProfile* curr_profile_{nullptr};
    rclcpp::Time start_time_;
    double vel_;
};

// =======================================================
// Node Class
// =======================================================
class DiffDriveBrain : public rclcpp::Node {

public:
    DiffDriveBrain() : Node("diff_drive_brain") {
        declareParams();
        getParams();

        publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>("/rwd_diff_controller/cmd_vel", 10);

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/get_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){ poseCallback(msg); });

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){ goalCallback(msg); });

        timer_ = create_wall_timer(50ms, [this](){
            geometry_msgs::msg::TwistStamped cmd;
            controller_->update(now(), cmd);
            publisher_->publish(cmd);
        });

        log_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&DiffDriveBrain::logStatus, this)
        );

    }

private:
    void declareParams() {
        this->declare_parameter<float>("p", 0.5);
        this->declare_parameter<float>("i", 0.0);
        this->declare_parameter<float>("d", 0.0);
        this->declare_parameter<float>("p_head", 0.2);
        this->declare_parameter<float>("i_head", 0.0);
        this->declare_parameter<float>("d_head", 0.0);
        this->declare_parameter<std::string>("control_mode", "pid");
    }

    void getParams() {
        this->get_parameter("p", k_p_);
        this->get_parameter("i", k_i_);
        this->get_parameter("d", k_d_);
        this->get_parameter("p_head", k_p_h_);
        this->get_parameter("i_head", k_i_h_);
        this->get_parameter("d_head", k_d_h_);
        this->get_parameter("control_mode", control_mode_);

        if (control_mode_ == "pid") {
            controller_ = std::make_unique<PIDController>(k_p_, k_i_, k_d_, k_p_h_, k_i_h_, k_d_h_);
        } else if (control_mode_ == "pid_profiler") {
            controller_ = std::make_unique<PIDProfilerController>(k_p_, k_i_, k_d_, k_p_h_, k_i_h_, k_d_h_);
        } else {
            controller_ = std::make_unique<MotionProfileController>();
        }
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.orientation.x, msg->pose.orientation.y,
            msg->pose.orientation.z, msg->pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        controller_->updatePose({msg->pose.position.x, msg->pose.position.y, yaw});
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.orientation.x, msg->pose.orientation.y,
            msg->pose.orientation.z, msg->pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        controller_->setTarget({msg->pose.position.x, msg->pose.position.y, yaw});
    }

    void logStatus()
    {
        if (!controller_) return;
        Pose2d curr = controller_->getCurrentPose();
        Pose2d goal = controller_->getGoalPose();
        double dist = controller_->getDistance();
        double vel = controller_->getVel();
        RCLCPP_INFO(
            this->get_logger(),
            "[%s] | Current Pose: x=%.2f y=%.2f heading=%.2f | Goal Pose: x=%.2f y=%.2f heading=%.2f | Distance=%.3f | Vel = %.3f",
            control_mode_.c_str(),
            static_cast<double>(curr.x), static_cast<double>(curr.y), static_cast<double>(curr.heading),
            static_cast<double>(goal.x), static_cast<double>(goal.y), static_cast<double>(goal.heading),
            static_cast<double>(dist), static_cast<double>(vel)
        );
        if (control_mode_ == "pid" or control_mode_ == "pid_profiler")
        {
            RCLCPP_INFO(this->get_logger(), "State: %s", controller_->getState().c_str());
        }

    }


    // Params
    float k_p_, k_i_, k_d_, k_p_h_, k_i_h_, k_d_h_;
    std::string control_mode_;

    // ROS2 interfaces
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Current controller
    std::unique_ptr<DiffDriveController> controller_;

    rclcpp::TimerBase::SharedPtr log_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveBrain>());
    rclcpp::shutdown();
    return 0;
}
