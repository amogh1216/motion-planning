#include <cmath>
#include <algorithm>

class TrapezoidalMotionProfile {
public:
    TrapezoidalMotionProfile(float max_vel, float max_acc, bool is_turn)
        : max_vel_(max_vel), max_acc_(max_acc), is_turn(is_turn), start_(0), goal_(0), duration_(0),
          accel_time_(0), cruise_time_(0), decel_time_(0), direction_(1) {}

    // Initialize profile for a given distance or heading
    void initialize(float distance) {
        goal_ = distance;
        float delta = goal_;
        direction_ = (delta >= 0) ? 1 : -1;
        float abs_delta = std::abs(delta);

        // Time to accelerate to max velocity
        accel_time_ = max_vel_ / max_acc_;
        float accel_dist = 0.5f * max_acc_ * accel_time_ * accel_time_;

        if (2 * accel_dist > abs_delta) {
            // Triangle profile (never reaches max velocity)
            accel_time_ = std::sqrt(2 * max_acc_ * abs_delta) / max_acc_; //std::sqrt(abs_delta / max_acc_);
            cruise_time_ = 0;
            decel_time_ = accel_time_;
        } else {
            // Trapezoidal profile
            cruise_time_ = (abs_delta - accel_dist - accel_dist) / max_vel_;
            decel_time_ = accel_time_;
        }
        duration_ = accel_time_ + cruise_time_ + decel_time_;
    }

    // Get position at time t (relative to start)
    float getPosition(float t) const {
        // float abs_delta = std::abs(goal_ - start_);
        if (t <= 0) return start_;
        if (t >= duration_) return goal_;

        float pos = 0;
        if (t < accel_time_) {
            pos = 0.5f * max_acc_ * t * t;
        } else if (t < (accel_time_ + cruise_time_)) {
            pos = 0.5f * max_acc_ * accel_time_ * accel_time_;
            pos += max_vel_ * (t - accel_time_);
        } else {
            float t_decel = t - accel_time_ - cruise_time_;
            pos = 0.5f * max_acc_ * accel_time_ * accel_time_;
            pos += max_vel_ * cruise_time_;
            pos += max_vel_ * t_decel - 0.5f * max_acc_ * t_decel * t_decel;
        }
        return start_ + direction_ * pos;
    }

    // Get velocity at time t
    float getVelocity(float t) const {
        if (t <= 0) return 0;
        if (t >= duration_) return 0;

        if (t < accel_time_) {
            return direction_ * max_acc_ * t;
        } else if (t < (accel_time_ + cruise_time_)) {
            return direction_ * max_vel_;
        } else {
            float t_decel = t - accel_time_ - cruise_time_;
            return direction_ * (max_vel_ - max_acc_ * t_decel);
        }
    }

    float getDuration() const { return duration_; }
    float getIsTurn() const {return is_turn; }

private:
    float max_vel_;
    float max_acc_;
    bool is_turn;
    float start_;
    float goal_;
    float duration_;
    float accel_time_;
    float cruise_time_;
    float decel_time_;
    int direction_;
};