#include "control/balance_controller/balance_controller.h"

#include <string>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <memory>
#include <shared_mutex>
#include <thread>

#include <yaml-cpp/yaml.h>

// ==================== TEMP TUNING INSTRUMENTATION ====================
#include <cstdlib>
#include <fstream>
// =======================================================================

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
    // ==================== TEMP TUNING INSTRUMENTATION ====================
    config_path_ = config_path;
    // =======================================================================

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

    kf_x_.setNoise(cfg_.kalman_x_process_noise_pos, cfg_.kalman_x_process_noise_vel, cfg_.kalman_x_measurement_noise);
    kf_y_.setNoise(cfg_.kalman_y_process_noise_pos, cfg_.kalman_y_process_noise_vel, cfg_.kalman_y_measurement_noise);
    kf_radius_.setNoise(cfg_.kalman_radius_process_noise_pos, cfg_.kalman_radius_process_noise_vel, cfg_.kalman_radius_measurement_noise);
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

// ==================== TEMP TUNING INSTRUMENTATION ====================
void BalanceController::reload_config(){
    YAML::Node config = YAML::LoadFile(config_path_)["control"]["balance_controller"];
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

    kf_x_.setNoise(cfg_.kalman_x_process_noise_pos, cfg_.kalman_x_process_noise_vel, cfg_.kalman_x_measurement_noise);
    kf_y_.setNoise(cfg_.kalman_y_process_noise_pos, cfg_.kalman_y_process_noise_vel, cfg_.kalman_y_measurement_noise);
    kf_radius_.setNoise(cfg_.kalman_radius_process_noise_pos, cfg_.kalman_radius_process_noise_vel, cfg_.kalman_radius_measurement_noise);

    std::cout << "[tuning] config reloaded from " << config_path_ << std::endl;
}

void BalanceController::plot_and_clear_log(){
    const std::string log_path = "/home/harsh/Documents/Projects/ball-balancing-table/.logs/tuning_xy.txt";
    const std::string script_path = "/home/harsh/Documents/Projects/ball-balancing-table/scripts/plot_tuning.py";

    int result = std::system(("python3 " + script_path + " " + log_path).c_str());
    if (result != 0) {
        std::cerr << "[tuning] plot_tuning.py failed (exit code " << result << ")" << std::endl;
    }

    std::ofstream clear_log(log_path, std::ios::trunc);
}
// =======================================================================

void BalanceController::running() {
    using Clock = std::chrono::high_resolution_clock;

    // ==================== TEMP TUNING INSTRUMENTATION ====================
    auto run_start = Clock::now();
    std::ofstream tuning_log(
        "/home/harsh/Documents/Projects/ball-balancing-table/.logs/tuning_xy.txt",
        std::ios::app
    );
    // =======================================================================

    while (!stop_flag.load()) {
        auto now = Clock::now();

        std::shared_lock<std::shared_mutex> shr_lock(ball_mtx_);
        bool stale_value = *ball_stale_;
        double measured_x = *ball_x_;
        double measured_y = *ball_y_;
        double measured_radius = *ball_radius_;
        int measured_frame = *ball_capture_frame_;
        shr_lock.unlock();

        if (stale_value) {
            roll_controller.reset();
            pitch_controller.reset();
            height_controller.reset();

            prev_time_.reset();
            prev_capture_frame_.reset();
            is_extrapolated_frame_ = false;

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
            if (!prev_capture_frame_.has_value()) {
                // First valid frame after (re)start: initialize the filters directly at the measurement.
                kf_x_.reset(measured_x);
                kf_y_.reset(measured_y);
                kf_radius_.reset(measured_radius);

                prev_capture_frame_ = measured_frame;
                prev_time_ = now;
                is_extrapolated_frame_ = false;
            }
            else {
                double dt = std::chrono::duration<double>(now - prev_time_.value()).count();

                kf_x_.predict(dt);
                kf_y_.predict(dt);
                kf_radius_.predict(dt);

                if (prev_capture_frame_.value() != measured_frame) {
                    kf_x_.update(measured_x);
                    kf_y_.update(measured_y);
                    kf_radius_.update(measured_radius);

                    prev_capture_frame_ = measured_frame;
                    is_extrapolated_frame_ = false;
                } else {
                    // No new camera frame since last tick: rely on the filter's own
                    // constant-velocity prediction instead of a new measurement.
                    is_extrapolated_frame_ = true;
                }

                prev_time_ = now;
            }

            double current_x = kf_x_.position();
            double current_y = kf_y_.position();
            double current_radius = kf_radius_.position();

            double vel_x = kf_x_.velocity();
            double vel_y = kf_y_.velocity();
            double vel_radius = kf_radius_.velocity();

            // ==================== TEMP LATENCY COMPENSATION EXPERIMENT ====================
            // Quick test: push the position estimate forward by a fixed guess at pipeline
            // latency, using the filter's own velocity. Delete this block (and switch the
            // two update() calls below back to current_x/current_y) to remove.
            constexpr double kLookaheadS = 0.00;
            double lookahead_x = current_x + vel_x * kLookaheadS;
            double lookahead_y = current_y + vel_y * kLookaheadS;
 	    double lookahead_radius = current_radius + (vel_radius * 1.0);          
 
 	   // =================================================================================

            float roll = roll_controller.update(
                lookahead_y,
                vel_y,
                cfg_.balance_controller_thread_delay_ms/1000.0,
                cfg_.PID_roll_derivative_deadband,
                cfg_.PID_roll_derivative_deadband_multi
            );

            float pitch = pitch_controller.update(
                lookahead_x,
                vel_x,
                cfg_.balance_controller_thread_delay_ms/1000.0,
                cfg_.PID_pitch_derivative_deadband,
                cfg_.PID_pitch_derivative_deadband_multi
            );

            float height = height_controller.update(
                cfg_.target_radius - lookahead_radius,
                -vel_radius,
                cfg_.balance_controller_thread_delay_ms/1000.0,
                cfg_.PID_height_derivative_deadband,
                cfg_.PID_height_derivative_deadband_multi
            );

            height = cfg_.target_radius - current_radius > cfg_.PID_height_threshold ? height : 0.0;

            // ==================== TEMP TUNING INSTRUMENTATION ====================
            double elapsed_s = std::chrono::duration<double>(now - run_start).count();
            tuning_log << elapsed_s << ","
                       << measured_x << "," << measured_y << ","
                       << current_x << "," << current_y << ","
                       << vel_x << "," << vel_y << ","
                       << pitch << "," << roll << ","
                       << measured_radius << "," << current_radius << ","
                       << vel_radius << "," << height << "\n";
            // =======================================================================

            std::cout << "measured_frame: " << measured_frame << std::endl;
            std::cout << "current_x: " << current_x << " current_y: " << current_y << " current_radius: " << current_radius  << std::endl;
            std::cout << "vel_x: " << vel_x
                      << " vel_y: " << vel_y
                      << " vel_radius: " << vel_radius << std::endl;
            std::cout << "extrapolated: " << is_extrapolated_frame_ << std::endl;
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
