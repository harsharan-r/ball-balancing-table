// balance_controller.h
#pragma once

#include "balance_controller_config.h"
#include "../action/servo.h"

#include <string>
#include <unordered_map>
#include <memory>
#include <shared_mutex>

class BalanceController{
public:

    BalanceController(const std::string& config_path,
                      Servo& servo_center, 
                      Servo& servo_left, 
                      Servo& servo_right,
                      std::shared_ptr<double> ball_x, 
                      std::shared_ptr<double> ball_y, 
                      std::shared_ptr<double> ball_radius,
                      std::shared_ptr<bool> ball_stale_,
                      std::shared_mutex& ball_mtx);
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

    std::shared_ptr<double> ball_x_;
    std::shared_ptr<double> ball_y_;
    std::shared_ptr<double> ball_radius_;
    std::shared_ptr<bool> ball_stale_;
    std::shared_mutex& ball_mtx_;

    BalanceControllerConfig cfg_;

};
