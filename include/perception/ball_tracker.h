#pragma once

#include "ball_tracker_config.h"

#include <memory>
#include <string>
#include <vector>
#include <shared_mutex>

#include <libcamera/libcamera.h>
#include <opencv2/core.hpp>

class BallTracker {
public:
    explicit BallTracker(const std::string& config_path, 
                         std::shared_ptr<double> ball_x, 
                         std::shared_ptr<double> ball_y, 
                         std::shared_ptr<double> ball_radius,
                         std::shared_mutex& ball_mtx);

    void startTracking();
    void startCalibration();
    void startCamera();
    void stopCamera();
    void shutdown();

    bool is_camera_started = false;
    bool is_calibrated = false;
    bool is_ball_values_stale = true;

    std::shared_ptr<double> ball_x_;
    std::shared_ptr<double> ball_y_;
    std::shared_ptr<double> ball_radius_;
    std::shared_mutex& ball_mtx_;

private:
    enum class Mode{
        Idle,
        Calibration,
        Track
    };

    void config_camera();
    void requestComplete(libcamera::Request *request);
    void calibrate(libcamera::Request *request, libcamera::FrameBuffer *buffer, libcamera::StreamConfiguration const &cfg);
    void calibrate_ball_colour(libcamera::FrameBuffer *buffer, libcamera::StreamConfiguration const &cfg);
    void track(libcamera::FrameBuffer *buffer, libcamera::StreamConfiguration const &cfg, bool save_frame=true);
    void detectPingPongBall(cv::Mat &frame, cv::Mat &mask);
    cv::Mat stream_buffer_to_bgrx(uint8_t *&ptr, libcamera::FrameBuffer *buffer, libcamera::StreamConfiguration const &cfg);

    BallTrackerConfig cfg_;

    std::unique_ptr<libcamera::CameraManager> cm_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream *stream_ = nullptr;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::unique_ptr<libcamera::ControlList> controls_;

    Mode mode = Mode::Idle;

    int calibration_frames = 0;

    int ball_hue_min;
    int ball_hue_max;
    int ball_sat_min;
    int ball_sat_max;
    int ball_value_min;
    int ball_value_max;
};