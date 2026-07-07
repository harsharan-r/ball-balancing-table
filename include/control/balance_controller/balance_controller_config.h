// balance_controller_config.h
#pragma once

#include <yaml-cpp/yaml.h>

struct BalanceControllerConfig {
  
  float arm_dist_from_center;
  float arm_to_arm_dist;

  float axis_offset;
  float lower_arm_length;
  float upper_arm_length;

  float servo_center_reference_offset;
  float servo_left_reference_offset;
  float servo_right_reference_offset;

  float servo_center_idle_height;
  float servo_left_idle_height;
  float servo_right_idle_height;

  float platform_ready_height;
  float platform_ready_roll;
  float platform_ready_pitch;

  float PID_roll_kp;
  float PID_roll_ki;
  float PID_roll_kd;
  float PID_roll_min;
  float PID_roll_max;
  float PID_roll_alpha;

  float PID_pitch_kp;
  float PID_pitch_ki;
  float PID_pitch_kd;
  float PID_pitch_min;
  float PID_pitch_max;
  float PID_pitch_alpha;

  float PID_derivative_deadband;

  BalanceControllerConfig() = default;

  explicit BalanceControllerConfig(YAML::Node config) {

    arm_dist_from_center = config["ARM_DIST_FROM_CENTER"].as<float>();
    arm_to_arm_dist = config["ARM_TO_ARM_DIST"].as<float>();
    
    axis_offset = config["AXIS_OFFSET"].as<float>();
    lower_arm_length = config["LOWER_ARM_LENGTH"].as<float>();
    upper_arm_length = config["UPPER_ARM_LENGTH"].as<float>();

    servo_center_reference_offset = config["SERVO_CENTER_REFERENCE_OFFSET"].as<float>();    
    servo_left_reference_offset = config["SERVO_LEFT_REFERENCE_OFFSET"].as<float>();      
    servo_right_reference_offset = config["SERVO_RIGHT_REFERENCE_OFFSET"].as<float>();    

    servo_center_idle_height = config["SERVO_CENTER_IDLE_HEIGHT"].as<float>();
    servo_left_idle_height = config["SERVO_LEFT_IDLE_HEIGHT"].as<float>();
    servo_right_idle_height = config["SERVO_RIGHT_IDLE_HEIGHT"].as<float>();
    
    platform_ready_height = config["PLATFORM_READY_HEIGHT"].as<float>();
    platform_ready_roll = config["PLATFORM_READY_ROLL"].as<float>();
    platform_ready_pitch = config["PLATFORM_READY_PITCH"].as<float>();

    PID_roll_kp = config["PID_ROLL_KP"].as<float>();
    PID_roll_ki = config["PID_ROLL_KI"].as<float>();
    PID_roll_kd = config["PID_ROLL_KD"].as<float>();
    PID_roll_min = config["PID_ROLL_MIN"].as<float>();
    PID_roll_max = config["PID_ROLL_MAX"].as<float>();
    PID_roll_alpha = config["PID_ROLL_ALPHA"].as<float>();

    PID_pitch_kp = config["PID_PITCH_KP"].as<float>();
    PID_pitch_ki = config["PID_PITCH_KI"].as<float>();
    PID_pitch_kd = config["PID_PITCH_KD"].as<float>();
    PID_pitch_min = config["PID_PITCH_MIN"].as<float>();
    PID_pitch_max = config["PID_PITCH_MAX"].as<float>();
    PID_pitch_alpha = config["PID_PITCH_ALPHA"].as<float>();

    PID_derivative_deadband = config["PID_DERIVATIVE_DEADBAND"].as<float>();
  }
};