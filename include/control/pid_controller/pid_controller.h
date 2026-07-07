// pid_controller.h 
#pragma once 

#include <optional>

class PIDController {
public:
    PIDController();

    float update(float error, float dt, float alpha, float derivative_deadband);
    void reset();
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min_output, float max_output);

private:
    float kp_{0.0f};
    float ki_{0.0f};
    float kd_{0.0f};

    std::optional<float> prev_error_;
    float integral_{0.0f};

    float min_output_{0.0f};
    float max_output_{0.0f};
};
