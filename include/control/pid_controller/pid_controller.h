// pid_controller.h 
#pragma once 

#include <optional>

class PIDController {
public:
    PIDController();

    float update(float error, float velocity, float dt, float derivative_deadband, float derivative_deadband_multi);
    void reset();
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min_output, float max_output);
    void setAntiWindup(bool integral_anti_windup, float integral_anti_windup_threshold);

private:

    float kp_{0.0f};
    float ki_{0.0f};
    float kd_{0.0f};

    float integral_{0.0f};

    float min_output_{0.0f};
    float max_output_{0.0f};

    bool integral_anti_windup_{false};
    float integral_anti_windup_threshold_;
};
