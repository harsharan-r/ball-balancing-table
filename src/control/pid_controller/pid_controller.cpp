#include "control/pid_controller/pid_controller.h"

#include <algorithm>
#include <cmath>
#include <iostream>

PIDController::PIDController(){}

float PIDController::update(float error, float velocity, float dt, float derivative_deadband, float derivative_deadband_multi){

    float proportional = error;
    float derivative = std::fabs(velocity) < derivative_deadband ? velocity*derivative_deadband_multi : velocity;
    std::cout << "Proportional: " << proportional << " Derivative: " << derivative << " Integral: " << integral_ << std::endl;

    integral_ += error*dt;
    if(integral_anti_windup_ && std::fabs(error) < integral_anti_windup_threshold_){
        integral_ = 0.0;
    } 

    float output = proportional*kp_ + integral_*ki_ + derivative*kd_;

    return std::clamp(output, min_output_, max_output_);
}

void PIDController::reset(){
    integral_ = 0.0;
}

void PIDController::setGains(float kp, float ki, float kd){
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(float min_output, float max_output){
    min_output_ = min_output;
    max_output_ = max_output;
}

void PIDController::setAntiWindup(bool integral_anti_windup, float integral_anti_windup_threshold){
    integral_anti_windup_ = integral_anti_windup;
    if(integral_anti_windup_){
        integral_anti_windup_threshold_ = integral_anti_windup_threshold;
    }
}
