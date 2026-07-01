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
  }
};