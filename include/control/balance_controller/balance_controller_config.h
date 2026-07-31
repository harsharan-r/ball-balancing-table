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

  int balance_controller_thread_delay_ms;

  float alpha_x;
  float alpha_y;
  float alpha_radius;

  float PID_roll_kp;
  float PID_roll_ki;
  float PID_roll_kd;
  float PID_roll_min;
  float PID_roll_max;

  float PID_pitch_kp;
  float PID_pitch_ki;
  float PID_pitch_kd;
  float PID_pitch_min;
  float PID_pitch_max;

  float PID_height_kp;
  float PID_height_ki;
  float PID_height_kd;
  float PID_height_min;
  float PID_height_max;
  float PID_height_threshold;

  float target_radius;

  float PID_roll_derivative_deadband;
  float PID_pitch_derivative_deadband;
  float PID_height_derivative_deadband;

  float PID_roll_derivative_deadband_multi;
  float PID_pitch_derivative_deadband_multi;
  float PID_height_derivative_deadband_multi;
  
  bool PID_anti_windup;
  float PID_anti_windup_threshold;

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

    balance_controller_thread_delay_ms = config["BALANCE_CONTROLLER_THREAD_DELAY_MS"].as<int>();

    alpha_x = config["ALPHA_X"].as<float>();
    alpha_y = config["ALPHA_Y"].as<float>();
    alpha_radius = config["ALPHA_RADIUS"].as<float>();

    PID_roll_kp = config["PID_ROLL_KP"].as<float>();
    PID_roll_ki = config["PID_ROLL_KI"].as<float>();
    PID_roll_kd = config["PID_ROLL_KD"].as<float>();
    PID_roll_min = config["PID_ROLL_MIN"].as<float>();
    PID_roll_max = config["PID_ROLL_MAX"].as<float>();

    PID_pitch_kp = config["PID_PITCH_KP"].as<float>();
    PID_pitch_ki = config["PID_PITCH_KI"].as<float>();
    PID_pitch_kd = config["PID_PITCH_KD"].as<float>();
    PID_pitch_min = config["PID_PITCH_MIN"].as<float>();
    PID_pitch_max = config["PID_PITCH_MAX"].as<float>();

    PID_height_kp = config["PID_HEIGHT_KP"].as<float>();
    PID_height_ki = config["PID_HEIGHT_KI"].as<float>();
    PID_height_kd = config["PID_HEIGHT_KD"].as<float>();
    PID_height_min = config["PID_HEIGHT_MIN"].as<float>();
    PID_height_max = config["PID_HEIGHT_MAX"].as<float>();
    PID_height_threshold = config ["PID_HEIGHT_THRESHOLD"].as<float>();

    target_radius = config["TARGET_RADIUS"].as<float>();

    PID_roll_derivative_deadband = config["PID_ROLL_DERIVATIVE_DEADBAND"].as<float>();
    PID_pitch_derivative_deadband = config["PID_PITCH_DERIVATIVE_DEADBAND"].as<float>();
    PID_height_derivative_deadband = config["PID_HEIGHT_DERIVATIVE_DEADBAND"].as<float>();

    PID_roll_derivative_deadband_multi = config["PID_ROLL_DERIVATIVE_DEADBAND_MULTI"].as<float>();
    PID_pitch_derivative_deadband_multi = config["PID_PITCH_DERIVATIVE_DEADBAND_MULTI"].as<float>();
    PID_height_derivative_deadband_multi = config["PID_HEIGHT_DERIVATIVE_DEADBAND_MULTI"].as<float>();

    PID_anti_windup = config["PID_ANTI_WINDUP"].as<bool>();
    PID_anti_windup_threshold = config["PID_ANTI_WINDUP_THRESHOLD"].as<float>();
  }
};