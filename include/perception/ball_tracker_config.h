// ball_tracker_config.h
#pragma once

#include <yaml-cpp/yaml.h>

static constexpr int MILLISECONDS_IN_SECOND = 1000000;

struct BallTrackerConfig {
    int camera_stream_index;
    int camera_stream_width;
    int camera_stream_height;
    int camera_frame_rate;
    int camera_min_frame_duration;
    int camera_max_frame_duration;
    bool camera_ae_enable; // TODO: delete
    int camera_ae_warmup_frames;
    int camera_exposure_time;

    int image_downsampling_factor;
    int image_crop_width;
    int image_crop_height;

    float calibration_margin;
    float calibration_x_px;
    float calibration_y_px;
    int calibration_radius_px;

    int ball_hue_min;
    int ball_hue_max;
    int ball_sat_min;
    int ball_sat_max;
    int ball_value_min;
    int ball_value_max;

    int hsv_hue_min;
    int hsv_sat_min;
    int hsv_value_min;
    int hsv_hue_max;
    int hsv_sat_max;
    int hsv_value_max;

    BallTrackerConfig() = default;

    explicit BallTrackerConfig(YAML::Node config) {
        camera_stream_index = config["CAMERA_STREAM_INDEX"].as<int>();
        camera_stream_width = config["CAMERA_STREAM_WIDTH"].as<int>();
        camera_stream_height = config["CAMERA_STREAM_HEIGHT"].as<int>();
        camera_frame_rate = config["CAMERA_FRAME_RATE"].as<int>();
        camera_min_frame_duration = MILLISECONDS_IN_SECOND / camera_frame_rate;
        camera_max_frame_duration = MILLISECONDS_IN_SECOND / camera_frame_rate;
        camera_ae_enable = config["CAMERA_AE_ENABLE"].as<bool>();  // TODO: delete
        camera_ae_warmup_frames = config["CAMERA_AE_WARMUP_FRAMES"].as<int>();
        camera_exposure_time = config["CAMERA_EXPOSURE_TIME"].as<int>();

        image_downsampling_factor = config["IMAGE_DOWNSAMPLING_FACTOR"].as<int>();
        image_crop_width = config["IMAGE_CROP_WIDTH"].as<int>();
        image_crop_height = config["IMAGE_CROP_HEIGHT"].as<int>();

        calibration_margin = config["CALIBRATION_MARGIN"].as<float>();
        calibration_x_px = config["CALIBRATION_X_PX"].as<float>();
        calibration_y_px = config["CALIBRATION_Y_PX"].as<float>();
        calibration_radius_px = config["CALIBRATION_RADIUS_PX"].as<int>();

        ball_hue_max = config["BALL_HUE_MAX"].as<int>();
        ball_hue_min = config["BALL_HUE_MIN"].as<int>();
        ball_sat_min = config["BALL_SAT_MIN"].as<int>();
        ball_sat_max = config["BALL_SAT_MAX"].as<int>();
        ball_value_min = config["BALL_VALUE_MIN"].as<int>();
        ball_value_max = config["BALL_VALUE_MAX"].as<int>();

        hsv_hue_min = config["HSV_HUE_MIN"].as<int>();
        hsv_sat_min = config["HSV_SAT_MIN"].as<int>();
        hsv_value_min = config["HSV_VALUE_MIN"].as<int>();
        hsv_hue_max = config["HSV_HUE_MAX"].as<int>();
        hsv_sat_max = config["HSV_SAT_MAX"].as<int>();
        hsv_value_max = config["HSV_VALUE_MAX"].as<int>();
    }
};