#include "control/balance_controller/balance_controller.h"

#include <string>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <memory>
#include <shared_mutex>
#include <thread>

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
    std::shared_mutex& ball_mtx,
    std::shared_ptr<int> ball_capture_frame
)
    : servo_center_(servo_center),
      servo_left_(servo_left),
      servo_right_(servo_right),
      ball_x_(ball_x),
      ball_y_(ball_y),
      ball_radius_(ball_radius),
      ball_stale_(ball_stale),
      ball_mtx_(ball_mtx),
      ball_capture_frame_(ball_capture_frame)
{
    YAML::Node config = YAML::LoadFile(config_path)["control"]["balance_controller"];
    cfg_ = BalanceControllerConfig(config);

    roll_controller.setGains(cfg_.PID_roll_kp, cfg_.PID_roll_ki, cfg_.PID_roll_kd);
    roll_controller.setOutputLimits(cfg_.PID_roll_min, cfg_.PID_roll_max);
    roll_controller.setAntiWindup(cfg_.PID_anti_windup, cfg_.PID_anti_windup_threshold);

    pitch_controller.setGains(cfg_.PID_pitch_kp, cfg_.PID_pitch_ki, cfg_.PID_pitch_kd);
    pitch_controller.setOutputLimits(cfg_.PID_pitch_min, cfg_.PID_pitch_max);
    pitch_controller.setAntiWindup(cfg_.PID_anti_windup, cfg_.PID_anti_windup_threshold);

    height_controller.setGains(cfg_.PID_height_kp, cfg_.PID_height_ki, cfg_.PID_height_kd);
    height_controller.setOutputLimits(cfg_.PID_height_min, cfg_.PID_height_max);
    height_controller.setAntiWindup(cfg_.PID_anti_windup, cfg_.PID_anti_windup_threshold);
}

BalanceController::~BalanceController() {
    stop_balancing_thread();
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

    // TODO make this cleaner logic: make it all a function
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

void BalanceController::start_balancing_thread(){
    if (runner_thread && runner_thread->joinable()) {
        return;
    }

    stop_flag.store(false);

    runner_thread = std::make_unique<std::thread>(&BalanceController::running, this);
}

void BalanceController::stop_balancing_thread(){
    if (runner_thread && runner_thread->joinable()) {
        stop_flag.store(true);
        
        runner_thread->join();
        runner_thread.reset(); 
    }
}

void BalanceController::running() {
    using Clock = std::chrono::high_resolution_clock;

    while (!stop_flag.load()) {
        auto now = Clock::now();

        std::shared_lock<std::shared_mutex> shr_lock(ball_mtx_);
        bool stale_value = *ball_stale_;
        float measured_x = *ball_x_;
        float measured_y = *ball_y_;
        float measured_radius = *ball_radius_;
        int measured_frame = *ball_capture_frame_;
        shr_lock.unlock();

        double filtered_x = prev_ball_x_ ? low_pass_filter(measured_x, prev_ball_x_.value(), cfg_.alpha_x) : measured_x;
        double filtered_y = prev_ball_y_ ? low_pass_filter(measured_y, prev_ball_y_.value(), cfg_.alpha_y) : measured_y; 
        double filtered_radius = prev_ball_radius_ ? low_pass_filter(measured_radius, prev_ball_radius_.value(), cfg_.alpha_radius) : measured_radius;

        if (stale_value) {
            roll_controller.reset();
            pitch_controller.reset();
            height_controller.reset();

            prev_ball_x_.reset();
            prev_ball_y_.reset();
            prev_ball_radius_.reset();
            prev_ball_vel_x_.reset();
            prev_ball_vel_y_.reset();
            prev_ball_vel_radius_.reset();
            prev_time_.reset();
            prev_capture_frame_.reset();

            float plat_roll_rad = deg_to_rad(cfg_.platform_ready_roll);
            float plat_pitch_rad = deg_to_rad(cfg_.platform_ready_pitch);

            std::unordered_map<std::string, float> heights =
                rpz_to_arm_heights(plat_roll_rad, plat_pitch_rad, cfg_.platform_ready_height);

            float servo_center_angle =
                height_to_servo_angle(heights["center_arm_height"], cfg_.servo_center_reference_offset);
            float servo_left_angle =
                height_to_servo_angle(heights["left_arm_height"], cfg_.servo_left_reference_offset);
            float servo_right_angle =
                height_to_servo_angle(heights["right_arm_height"], cfg_.servo_right_reference_offset);

            servo_center_.setAngle(servo_center_angle);
            servo_left_.setAngle(servo_left_angle);
            servo_right_.setAngle(servo_right_angle);
        }
        else {
            float current_x = filtered_x;
            float current_y = filtered_y;
            float current_radius = filtered_radius;

            if (!prev_capture_frame_.has_value()) {
                prev_ball_x_ = filtered_x;
                prev_ball_y_ = filtered_y;
                prev_ball_radius_ = filtered_radius;
                prev_ball_vel_x_ = 0.0f;
                prev_ball_vel_y_ = 0.0f;
                prev_ball_vel_radius_ = 0.0f;
                prev_capture_frame_ = measured_frame;
                prev_time_ = now;
            }
            else if (prev_capture_frame_.value() != measured_frame) {
                double dt = std::chrono::duration<double>(now - prev_time_.value()).count();

                if (dt > 1e-6) {
                    prev_ball_vel_x_ = (filtered_x - prev_ball_x_.value()) / dt;
                    prev_ball_vel_y_ = (filtered_y - prev_ball_y_.value()) / dt;
                    prev_ball_vel_radius_ = (filtered_radius - prev_ball_radius_.value()) / dt;
                } else {
                    prev_ball_vel_x_ = 0.0f;
                    prev_ball_vel_y_ = 0.0f;
                    prev_ball_vel_radius_ = 0.0f;
                }

                current_x = filtered_x;
                current_y = filtered_y;
                current_radius = filtered_radius;

                prev_ball_x_ = filtered_x;
                prev_ball_y_ = filtered_y;
                prev_ball_radius_ = filtered_radius;
                prev_capture_frame_ = measured_frame;
                prev_time_ = now;
            }
            else {
                double dt = std::chrono::duration<double>(now - prev_time_.value()).count();

                current_x = prev_ball_x_.value() + prev_ball_vel_x_.value_or(0.0f) * dt;
                current_y = prev_ball_y_.value() + prev_ball_vel_y_.value_or(0.0f) * dt;
                current_radius = prev_ball_radius_.value() + prev_ball_vel_radius_.value_or(0.0f) * dt;
            }

            float roll = roll_controller.update(
                current_y,
                cfg_.balance_controller_thread_delay_ms/1000.0,
                cfg_.PID_roll_derivative_deadband,
                cfg_.PID_roll_derivative_deadband_multi
            );

            float pitch = pitch_controller.update(
                current_x,
                cfg_.balance_controller_thread_delay_ms/1000.0,
                cfg_.PID_pitch_derivative_deadband,
                cfg_.PID_pitch_derivative_deadband_multi
            );

            float height = height_controller.update(
                cfg_.target_radius - current_radius,
                cfg_.balance_controller_thread_delay_ms/1000.0,
                cfg_.PID_height_derivative_deadband,
                cfg_.PID_height_derivative_deadband_multi
            );

            height = cfg_.target_radius - current_radius > cfg_.PID_height_threshold ? height : 0.0;

            std::cout << "measured_frame: " << measured_frame << std::endl;
            std::cout << "current_x: " << current_x << " current_y: " << current_y << " current_radius: " << current_radius  << std::endl;
            std::cout << "vel_x: " << prev_ball_vel_x_.value_or(0.0f)
                      << " vel_y: " << prev_ball_vel_y_.value_or(0.0f) 
                      << " vel_radius: " << prev_ball_vel_radius_.value_or(0.0f) << std::endl;
            std::cout << "roll pid: " << roll << std::endl;
            std::cout << "pitch pid: " << pitch << std::endl;
            std::cout << "height pid: " << height << std::endl;

            float plat_roll_rad = deg_to_rad(cfg_.platform_ready_roll + roll);
            float plat_pitch_rad = deg_to_rad(cfg_.platform_ready_pitch + pitch);
            float plat_height = cfg_.platform_ready_height + height;

            std::unordered_map<std::string, float> heights =
                rpz_to_arm_heights(plat_roll_rad, plat_pitch_rad, plat_height);

            float servo_center_angle =
                height_to_servo_angle(heights["center_arm_height"], cfg_.servo_center_reference_offset);
            float servo_left_angle =
                height_to_servo_angle(heights["left_arm_height"], cfg_.servo_left_reference_offset);
            float servo_right_angle =
                height_to_servo_angle(heights["right_arm_height"], cfg_.servo_right_reference_offset);

            servo_center_.setAngle(servo_center_angle);
            servo_left_.setAngle(servo_left_angle);
            servo_right_.setAngle(servo_right_angle);
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(cfg_.balance_controller_thread_delay_ms)
        );
    }
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

double BalanceController::low_pass_filter(double value, double prev_value, float alpha){
    return alpha * value + (1.0 - alpha)*prev_value;
}
