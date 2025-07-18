#include <cmath>

class DiffPID {
public:
    // Default constructor
    DiffPID()
      : k_p_(0), k_i_(0), k_d_(0),
        k_p_head_(0), k_i_head_(0), k_d_head_(0),
        x_err_integral_(0), heading_err_integral_(0),
        prev_dist_err_(0), prev_heading_(0),
        curr_x_(0), curr_y_(0), curr_heading_(0),
        goal_x_(0), goal_y_(0), goal_heading_(0), goal_dist_(0), init_x_(0), init_y_(0), movingForward(false)
    {}

    // Parameterized constructor
    DiffPID(float k_p, float k_i, float k_d,
            float k_p_head, float k_i_head, float k_d_head)
      : k_p_(k_p), k_i_(k_i), k_d_(k_d),
        k_p_head_(k_p_head), k_i_head_(k_i_head), k_d_head_(k_d_head),
        x_err_integral_(0), heading_err_integral_(0),
        prev_dist_err_(0), prev_heading_(0),
        curr_x_(0), curr_y_(0), curr_heading_(0),
        goal_x_(0), goal_y_(0), goal_heading_(0), goal_dist_(0), init_x_(0), init_y_(0), movingForward(false)
    {}

    // Set the robot's current position (x, y) and heading (in radians)
    void updatePose(float x, float y, float heading) {
        curr_x_ = x;
        curr_y_ = y;

        prev_heading_ = curr_heading_;
        curr_heading_ = heading;
    }

    // Set the target and automatically calculate goal heading and distance
    void setTarget(float x, float y) {
        goal_x_ = x;
        goal_y_ = y;

        // reset goal distance
        init_x_ = curr_x_;
        init_y_ = curr_y_;
        float dx = goal_x_ - curr_x_;
        float dy = goal_y_ - curr_y_;
        goal_dist_ = std::sqrt(dx * dx + dy * dy);  // Distance to target
        prev_dist_err_ = goal_dist_;

        goal_heading_ = std::atan2(dy, dx);  // Angle to target
    }

    // Compute linear velocity based on distance error
    float computeLinearVelocity(float dt) {
        // curr distance from init point
        float dist = getDist();
        float distErr = goal_dist_ - dist;
        x_err_integral_ += distErr * dt;
        float derivative = dt > 0 ? (distErr - prev_dist_err_) / dt : 0;
        prev_dist_err_ = distErr;

        if (distErr > 0) movingForward = true;
        else movingForward = false;

        return (k_p_ * distErr) + (k_d_ * derivative) + (k_i_ * x_err_integral_);
    }

    // Compute angular velocity based on heading error
    float computeAngularVelocity(float dt) {
        float heading_err = goal_heading_ - curr_heading_;
        // Normalize error to [-pi, pi]
        if (heading_err > M_PI) heading_err -= 2*M_PI;
        else if (heading_err < -M_PI) heading_err += 2*M_PI;

        heading_err_integral_ += heading_err * dt;
        float derivative = dt > 0 ? (curr_heading_ - prev_heading_) / dt : 0;
        
        return (k_p_head_ * heading_err) + (k_i_head_ * heading_err_integral_) + (k_d_head_ * derivative);
    }

    // Update PID constants at runtime
    void updateGains(float k_p, float k_i, float k_d,
                     float k_p_head, float k_i_head, float k_d_head)
    {
        k_p_ = k_p;
        k_i_ = k_i;
        k_d_ = k_d;
        k_p_head_ = k_p_head;
        k_i_head_ = k_i_head;
        k_d_head_ = k_d_head;
    }

    void resetGoalDistance() {
        float dx = goal_x_ - curr_x_;
        float dy = goal_y_ - curr_y_;
        init_x_ = curr_x_;
        init_y_ = curr_y_;
        goal_dist_ = std::sqrt(dx * dx + dy * dy);  // Distance to target   
        goal_heading_ = std::atan2(dy, dx); // Angle to target
    }

    // Getters for current goal heading and distance (utility)
    float getGoalHeading() const { return goal_heading_; }
    float getGoalDistance() const { return goal_dist_; }
    float getHeadingErr() const {return goal_heading_ - curr_heading_;}
    float isMovingForward() const {return movingForward;}

    // distance from init point
    float getDist() const {
        float dx = curr_x_ - init_x_;
        float dy = curr_y_ - init_y_;
        return std::sqrt(dx*dx + dy*dy);
    }

private:
    // Linear PID coefficients
    float k_p_, k_i_, k_d_;
    // Heading PID coefficients
    float k_p_head_, k_i_head_, k_d_head_;
    // State variables for PID
    float x_err_integral_;
    float heading_err_integral_;
    float prev_dist_err_;
    float prev_heading_;
    // Pose and target state
    float curr_x_, curr_y_, curr_heading_;
    float goal_x_, goal_y_, goal_heading_, goal_dist_;
    float init_x_, init_y_;
    bool movingForward;
};
