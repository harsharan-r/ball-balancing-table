// balance_controller.h
#pragma once

#include "balance_controller_config.h"
#include "action/servo/servo.h"
#include "control/pid_controller/pid_controller.h"
#include "control/kalman_filter/kalman_filter.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>

class BalanceController{
public:

    BalanceController(const std::string& config_path,
                      Servo& servo_center,
                      Servo& servo_left,
                      Servo& servo_right,
                      std::shared_ptr<double> ball_x,
                      std::shared_ptr<double> ball_y,
                      std::shared_ptr<double> ball_radius,
                      std::shared_ptr<bool> ball_stale,
                      std::shared_mutex& ball_mtx,
                      std::shared_ptr<int> ball_capture_frame);
    ~BalanceController();

    void idle();
    void ready();

    void start_balancing_thread();
    void stop_balancing_thread();

    // ==================== TEMP TUNING INSTRUMENTATION ====================
    // Delete this block, the matching block in balance_controller.cpp, the
    // call sites in main.cpp, and scripts/plot_tuning.py to remove.
    void reload_config();
    void plot_and_clear_log();
    // =======================================================================

private:

    void running();

    float height_to_servo_angle(float height, float motor_ref_offset);
    std::unordered_map<std::string, float> rpz_to_arm_heights(float roll, float pitch, float center_z);
    double rad_to_deg(double rad);
    double deg_to_rad(double deg);

    Servo& servo_center_;
    Servo& servo_left_;
    Servo& servo_right_;

    PIDController roll_controller;
    PIDController pitch_controller;
    PIDController height_controller;

    std::shared_ptr<double> ball_x_;
    std::shared_ptr<double> ball_y_;
    std::shared_ptr<double> ball_radius_;
    std::shared_ptr<bool> ball_stale_;
    std::shared_mutex& ball_mtx_;
    std::shared_ptr<int> ball_capture_frame_;

    BalanceControllerConfig cfg_;

    KalmanFilter1D kf_x_;
    KalmanFilter1D kf_y_;
    KalmanFilter1D kf_radius_;

    std::optional<int> prev_capture_frame_;
    std::optional<std::chrono::high_resolution_clock::time_point> prev_time_;
    bool is_extrapolated_frame_ = false;

    std::unique_ptr<std::thread> runner_thread;
    std::atomic<bool> stop_flag{false};

    // ==================== TEMP TUNING INSTRUMENTATION ====================
    std::string config_path_;
    // =======================================================================
};
