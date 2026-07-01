// balance_controller.h
#pragma once

#include "balance_controller_config.h"
#include "../action/servo.h"

#include <string>
#include <unordered_map>

class BalanceController{
public:

    BalanceController(const std::string& config_path,
                      Servo& servo_center, 
                      Servo& servo_left, 
                      Servo& servo_right);
    void idle();
    void ready();
    void running();

private:
    
    float height_to_servo_angle(float height, float motor_ref_offset);
    std::unordered_map<std::string, float> rpz_to_arm_heights(float roll, float pitch, float center_z);
    double rad_to_deg(double rad);
    double deg_to_rad(double deg);
    
    Servo& servo_center_;
    Servo& servo_left_;
    Servo& servo_right_;

    BalanceControllerConfig cfg_;

};
