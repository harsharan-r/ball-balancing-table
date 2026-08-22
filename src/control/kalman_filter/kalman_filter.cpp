// kalman_filter.cpp
#include "control/kalman_filter/kalman_filter.h"

void KalmanFilter1D::setNoise(double process_noise_pos, double process_noise_vel, double measurement_noise) {
    q_pos_ = process_noise_pos;
    q_vel_ = process_noise_vel;
    r_ = measurement_noise;
}

void KalmanFilter1D::reset(double initial_position) {
    p_ = initial_position;
    v_ = 0.0;
    Ppp_ = 1.0;
    Ppv_ = 0.0;
    Pvv_ = 1.0;
}

void KalmanFilter1D::predict(double dt) {
    p_ = p_ + v_ * dt;
    // v_ unchanged: constant-velocity model.

    double Ppp_pred = Ppp_ + 2.0 * dt * Ppv_ + dt * dt * Pvv_ + q_pos_ * dt;
    double Ppv_pred = Ppv_ + dt * Pvv_;
    double Pvv_pred = Pvv_ + q_vel_ * dt;

    Ppp_ = Ppp_pred;
    Ppv_ = Ppv_pred;
    Pvv_ = Pvv_pred;
}

void KalmanFilter1D::update(double measurement) {
    double innovation = measurement - p_;
    double S = Ppp_ + r_;

    double Kp = Ppp_ / S;
    double Kv = Ppv_ / S;

    p_ = p_ + Kp * innovation;
    v_ = v_ + Kv * innovation;

    double Ppp_new = (1.0 - Kp) * Ppp_;
    double Ppv_new = (1.0 - Kp) * Ppv_;
    double Pvv_new = Pvv_ - Kv * Ppv_;

    Ppp_ = Ppp_new;
    Ppv_ = Ppv_new;
    Pvv_ = Pvv_new;
}
