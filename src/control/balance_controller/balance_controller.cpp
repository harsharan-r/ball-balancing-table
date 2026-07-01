#include "../../../include/control/balance_controller.h"

#include <string>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <memory>
#include <shared_mutex>

#include <yaml-cpp/yaml.h>

BalanceController::BalanceController(
    const std::string& config_path,
    Servo& servo_center,
    Servo& servo_left,
    Servo& servo_right,
    std::shared_ptr<double> ball_x,
    std::shared_ptr<double> ball_y,
    std::shared_ptr<double> ball_radius,
    std::shared_ptr<bool> ball_stale,
    std::shared_mutex& ball_mtx
)
    : servo_center_(servo_center),
      servo_left_(servo_left),
      servo_right_(servo_right),
      ball_x_(ball_x),
      ball_y_(ball_y),
      ball_radius_(ball_radius),
      ball_stale_(ball_stale),
      ball_mtx_(ball_mtx)
{
    YAML::Node config = YAML::LoadFile(config_path)["control"]["balance_controller"];
    cfg_ = BalanceControllerConfig(config);
}

void BalanceController::idle(){

  float servo_center_angle = height_to_servo_angle(cfg_.servo_center_idle_height, cfg_.servo_center_reference_offset);
  float servo_left_angle = height_to_servo_angle(cfg_.servo_left_idle_height, cfg_.servo_left_reference_offset);
  float servo_right_angle = height_to_servo_angle(cfg_.servo_right_idle_height, cfg_.servo_right_reference_offset);

  servo_center_.setAngle(servo_center_angle);
  servo_left_.setAngle(servo_left_angle);
  servo_right_.setAngle(servo_right_angle);
}

void BalanceController::ready(){

  float roll = deg_to_rad(cfg_.platform_ready_roll);
  float pitch = deg_to_rad(cfg_.platform_ready_pitch);
  float height = cfg_.platform_ready_height;

  std::unordered_map<std::string, float> heights = rpz_to_arm_heights(roll, pitch, height);

  float servo_center_angle = height_to_servo_angle(heights["center_arm_height"], cfg_.servo_center_reference_offset);
  float servo_left_angle = height_to_servo_angle(heights["left_arm_height"], cfg_.servo_left_reference_offset);
  float servo_right_angle = height_to_servo_angle(heights["right_arm_height"], cfg_.servo_right_reference_offset);

  servo_center_.setAngle(servo_center_angle);
  servo_left_.setAngle(servo_left_angle);
  servo_right_.setAngle(servo_right_angle);
}

void BalanceController::running(){

}

std::unordered_map<std::string, float> BalanceController::rpz_to_arm_heights(float roll, float pitch, float center_z)
{
    float center_arm_height =
        center_z + cfg_.arm_dist_from_center * std::tan(roll);

    float left_arm_height =
        (3.0f * center_z / 2.0f) - (center_arm_height / 2.0f) +
        cfg_.arm_to_arm_dist / 2.0f * std::tan(pitch);

    float right_arm_height =
        3.0f * center_z - center_arm_height - left_arm_height;

    return {
        {"center_arm_height", center_arm_height},
        {"left_arm_height", left_arm_height},
        {"right_arm_height", right_arm_height}
    };
}

float BalanceController::height_to_servo_angle(float height, float motor_ref_offset)
{

    float angle =
        std::atan(cfg_.axis_offset / height) +
        std::acos(
            (
                std::pow(cfg_.lower_arm_length, 2) +
                std::pow(cfg_.axis_offset, 2) +
                std::pow(height, 2) -
                std::pow(cfg_.upper_arm_length, 2)
            ) /
            (
                2.0f * cfg_.lower_arm_length *
                std::sqrt(std::pow(cfg_.axis_offset, 2) + std::pow(height, 2))
            )
        );

    return rad_to_deg(angle) + motor_ref_offset;  
}

double BalanceController::rad_to_deg(double rad){
    return rad * 180.0 / M_PI;
}

double BalanceController::deg_to_rad(double deg){
    return deg * M_PI / 180.0;
}