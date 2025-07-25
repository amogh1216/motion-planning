#include "diff_pid.cpp"
#include <algorithm>
#include <cmath>

class DiffPIDProfiler : public DiffPID {
public:

    DiffPIDProfiler()
      : max_linear_velocity_(0),
          max_linear_acceleration_(0),
          max_angular_velocity_(0),
          max_angular_acceleration_(0),
          prev_linear_vel_(0.0f),
          prev_angular_vel_(0.0f)
    {}

    DiffPIDProfiler(
        float k_p, float k_i, float k_d,
        float k_p_head, float k_i_head, float k_d_head,
        float max_lin_vel, float max_lin_acc,
        float max_ang_vel, float max_ang_acc)
        : DiffPID(k_p, k_i, k_d, k_p_head, k_i_head, k_d_head),
          max_linear_velocity_(max_lin_vel),
          max_linear_acceleration_(max_lin_acc),
          max_angular_velocity_(max_ang_vel),
          max_angular_acceleration_(max_ang_acc),
          prev_linear_vel_(0.0f),
          prev_angular_vel_(0.0f)
    {}

    // Compute linear velocity with velocity and acceleration limits
    float computeLinearVelocity(float dt, float prev_vel) {
        float raw_vel = DiffPID::computeLinearVelocity(dt);
        float acc = (raw_vel - prev_vel) / std::max(dt, 1e-6f);

        // Clamp acceleration
        if (acc > max_linear_acceleration_)
            raw_vel = prev_vel + max_linear_acceleration_ * dt;
        else if (acc < -max_linear_acceleration_)
            raw_vel = prev_vel - max_linear_acceleration_ * dt;

        // Clamp velocity
        raw_vel = std::clamp(raw_vel, -max_linear_velocity_, max_linear_velocity_);

        prev_linear_vel_ = raw_vel;
        return raw_vel;
    }

    // Compute angular velocity with velocity and acceleration limits
    float computeAngularVelocity(float dt, float prev_vel) {
        float raw_vel = DiffPID::computeAngularVelocity(dt);
        float acc = (raw_vel - prev_vel) / std::max(dt, 1e-6f);

        // Clamp acceleration
        if (acc > max_angular_acceleration_)
            raw_vel = prev_vel + max_angular_acceleration_ * dt;
        else if (acc < -max_angular_acceleration_)
            raw_vel = prev_vel - max_angular_acceleration_ * dt;

        // Clamp velocity
        raw_vel = std::clamp(raw_vel, -max_angular_velocity_, max_angular_velocity_);

        prev_angular_vel_ = raw_vel;
        return raw_vel;
    }

    void setMaxLinearVelocity(float v) { max_linear_velocity_ = v; }
    void setMaxLinearAcceleration(float a) { max_linear_acceleration_ = a; }
    void setMaxAngularVelocity(float v) { max_angular_velocity_ = v; }
    void setMaxAngularAcceleration(float a) { max_angular_acceleration_ = a; }

    float getPrevLinearVel() const { return prev_linear_vel_; }
    float getPrevAngularVel() const { return prev_angular_vel_; }

private:
    float max_linear_velocity_;
    float max_linear_acceleration_;
    float max_angular_velocity_;
    float max_angular_acceleration_;
    float prev_linear_vel_;
    float prev_angular_vel_;
};