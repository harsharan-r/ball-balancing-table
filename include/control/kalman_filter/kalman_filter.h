// kalman_filter.h
#pragma once

// 1D constant-velocity Kalman filter: tracks a scalar position and its
// velocity from noisy, irregularly-timed position measurements.
class KalmanFilter1D {
public:
    KalmanFilter1D() = default;

    void setNoise(double process_noise_pos, double process_noise_vel, double measurement_noise);

    // Reinitializes state to a known position with zero velocity.
    void reset(double initial_position);

    // Propagates state and uncertainty forward by dt (no new measurement).
    void predict(double dt);

    // Fuses a new position measurement into the state.
    void update(double measurement);

    double position() const { return p_; }
    double velocity() const { return v_; }

private:
    double q_pos_ = 0.0;
    double q_vel_ = 0.0;
    double r_ = 1.0;

    double p_ = 0.0;
    double v_ = 0.0;

    // Covariance matrix [[Ppp, Ppv], [Ppv, Pvv]]
    double Ppp_ = 1.0;
    double Ppv_ = 0.0;
    double Pvv_ = 1.0;
};
