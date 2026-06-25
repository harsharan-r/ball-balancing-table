#include "../../../include/perception/ball_tracker.h"

#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

#include <sys/mman.h>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

using namespace libcamera;

BallTracker::BallTracker(const std::string& config_path)
    : cm_(std::make_unique<CameraManager>())
{
    YAML::Node config = YAML::LoadFile(config_path)["perception"]["camera"];
    cfg_ = CameraConfig(config);

    ball_hue_min = cfg_.ball_hue_min;
    ball_hue_max = cfg_.ball_hue_max;
    ball_sat_min = cfg_.ball_sat_min;
    ball_sat_max = cfg_.ball_sat_max;
    ball_value_min = cfg_.ball_value_min;
    ball_value_max = cfg_.ball_value_max;

    config_camera();
}

void BallTracker::config_camera() {
    if (cm_->start() < 0) {
        throw std::runtime_error("Failed to start CameraManager");
    }

    auto cameras = cm_->cameras();
    if (cameras.empty()) {
        throw std::runtime_error("No cameras found");
    }

    camera_ = cm_->get(cameras[0]->id());
    if (!camera_) {
        throw std::runtime_error("Failed to get camera");
    }

    if (camera_->acquire() < 0) {
        throw std::runtime_error("Failed to acquire camera");
    }

    auto config = camera_->generateConfiguration({ StreamRole::Viewfinder });
    if (!config || config->size() != 1) {
        throw std::runtime_error("Failed to create libcamera config");
    }

    StreamConfiguration &streamConfig = config->at(cfg_.camera_stream_index);
    streamConfig.size.width = cfg_.camera_stream_width;
    streamConfig.size.height = cfg_.camera_stream_height;
    config->validate();

    if (camera_->configure(config.get()) < 0) {
        throw std::runtime_error("Failed to set libcamera config");
    }

    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    stream_ = streamConfig.stream();

    if (allocator_->allocate(stream_) < 0) {
        throw std::runtime_error("Failed to allocate memory for libcamera");
    }

    camera_->requestCompleted.connect(this, &BallTracker::requestComplete);

    controls_ = std::make_unique<ControlList>();
    controls_->set(
        controls::FrameDurationLimits,
        Span<const int64_t, 2>({
            cfg_.camera_min_frame_duration,
            cfg_.camera_max_frame_duration
        })
    );

    controls_->set(controls::ExposureTime, cfg_.camera_exposure_time);
}

void BallTracker::requestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled) {
        return;
    }

    for (const auto &p : request->buffers()) {
        FrameBuffer *buffer = p.second;
        const StreamConfiguration &cfg = p.first->configuration();
        const FrameMetadata &meta = buffer->metadata();

        (void)buffer;
        (void)cfg;

        if(mode == Mode::Calibration){
            calibrate(request, buffer, cfg);
        }
        else if(mode == Mode::Track){
            // TODO: Add tracking code
        }

        std::cout << "Frame seq=" << meta.sequence << std::endl;
    }

    if (!is_camera_started) {
        return;
    }

    request->reuse(Request::ReuseBuffers);

    if (camera_->queueRequest(request) < 0) {
        throw std::runtime_error("Failed to queue request");
    }
}

void BallTracker::calibrate(libcamera::Request *request, libcamera::FrameBuffer *buffer, StreamConfiguration const &cfg){
    if (is_calibrated) {
        return;
    }

    if (calibration_frames < cfg_.camera_ae_warmup_frames) {
        calibration_frames++;
        std::cout << "Warming up frames" << std::endl;
        return;
    }

    if (calibration_frames == cfg_.camera_ae_warmup_frames) {
        const ControlList &metadata = request->metadata();
        auto exposure_opt = metadata.get(libcamera::controls::ExposureTime);

        if (!exposure_opt.has_value()) {
            throw std::runtime_error("ExposureTime not found in metadata");
        }

        int32_t exposure_time = exposure_opt.value();

        request->controls().set(libcamera::controls::AeEnable, false);
        request->controls().set(libcamera::controls::ExposureTime, exposure_time);

        calibration_frames++;
        std::cout << "Locking exposure" << std::endl;
        return;
    }

    calibrate_ball_colour(buffer, cfg);
    std::cout << "Calibrated colour" << std::endl;
    is_calibrated = true;
}

void BallTracker::calibrate_ball_colour(libcamera::FrameBuffer *buffer, libcamera::StreamConfiguration const &cfg){

  uint8_t *frame_data;

  cv::Mat bgrxFrame = stream_buffer_to_bgrx(frame_data, buffer, cfg);

  cv::Mat bgrFrame;
  cv::cvtColor(bgrxFrame, bgrFrame, cv::COLOR_BGRA2BGR);                    
  
  cv::Mat hsvFrame;                                                             
  cv::cvtColor(bgrFrame, hsvFrame, cv::COLOR_BGR2HSV);          

  cv::Point2f center(cfg_.calibration_x_px, cfg_.calibration_y_px);
  cv::Mat mask = cv::Mat::zeros(hsvFrame.size(), CV_8UC1);
  cv::circle(mask, center, cfg_.calibration_radius_px, 255, -1);

  cv::Scalar mean, stddev;
  cv::meanStdDev(hsvFrame, mean, stddev, mask);

  int hue_min = mean[0] - cfg_.calibration_margin * stddev[0];
  int hue_max = mean[0] + cfg_.calibration_margin * stddev[0];
  int sat_min = mean[1] - cfg_.calibration_margin * stddev[1];
  int sat_max = mean[1] + cfg_.calibration_margin * stddev[1];
  int value_min = mean[2] - cfg_.calibration_margin * stddev[2];
  int value_max = mean[2] + cfg_.calibration_margin * stddev[2];

  ball_hue_min = std::clamp(hue_min, cfg_.hsv_hue_min, cfg_.hsv_hue_max);
  ball_hue_max = std::clamp(hue_max, cfg_.hsv_hue_min, cfg_.hsv_hue_max);
  ball_sat_min = std::clamp(sat_min, cfg_.hsv_sat_min, cfg_.hsv_sat_max);
  ball_sat_max = std::clamp(sat_max, cfg_.hsv_sat_min, cfg_.hsv_sat_max);
  ball_value_min = std::clamp(value_min, cfg_.hsv_value_min, cfg_.hsv_value_max);
  ball_value_max = std::clamp(value_max, cfg_.hsv_value_min, cfg_.hsv_value_max);

  std::cout << "HSV MIN: " << ball_hue_min << ", " 
                           << ball_sat_min << ", "
                           << ball_value_min << ", "
                           << std::endl;

  std::cout << "HSV MAX: " << ball_hue_max << ", "
                           << ball_sat_max << ", "
                           << ball_value_max << ", "
                           << std::endl;

    // TODO: get rid of this
  cv::Mat masked_result;
  bgrFrame.copyTo(masked_result, mask);
  cv::imwrite("masked_frame.jpg", masked_result);

  munmap(frame_data, buffer->planes()[0].length); 
}


cv::Mat BallTracker::stream_buffer_to_bgrx(uint8_t *&ptr,
			      FrameBuffer *buffer,
			      StreamConfiguration const &cfg){

  int fd = buffer->planes()[0].fd.get();                                        
  ptr = static_cast<uint8_t *>(                                        
    mmap(NULL,                                                                  
         buffer->planes()[0].length,                                            
         PROT_READ | PROT_WRITE,                                                
         MAP_SHARED,                                                            
         fd,                                                                    
         0));                                                                            
                                                                                
  cv::Mat bgrxFrame(cfg.size.height, cfg.size.width, CV_8UC4, ptr, cfg.stride); 
 
  return bgrxFrame;
}

void BallTracker::startTracking(){
    mode = Mode::Track;
    startCamera();
}

void BallTracker::startCalibration(){
    is_calibrated = false;
    mode = Mode::Calibration;
    startCamera();
}

void BallTracker::startCamera() {
    
    const auto &buffers = allocator_->buffers(stream_);

    requests_.clear();
    for (const auto &buffer : buffers) {
        auto request = camera_->createRequest();
        if (!request) {
            throw std::runtime_error("Failed to make request to libcamera");
        }

        if (request->addBuffer(stream_, buffer.get()) < 0) {
            throw std::runtime_error("Failed to add allocated buffer to libcamera request");
        }

        requests_.push_back(std::move(request));
    }

    if (!camera_) {
        throw std::runtime_error("Camera not configured");
    }

    if (camera_->start(controls_.get()) < 0) {
        throw std::runtime_error("Failed to start camera");
    }

    is_camera_started = true;

    for (auto &request : requests_) {
        if (camera_->queueRequest(request.get()) < 0) {
            throw std::runtime_error("Failed to queue request");
        }
    }

}

void BallTracker::stopCamera() {
    if (!camera_ || !is_camera_started) {
        return;
    }

    is_camera_started = false;
    camera_->stop();
}

void BallTracker::shutdown(){
    stopCamera();

    if (allocator_ && stream_) {
        allocator_->free(stream_);
    }

    allocator_.reset();

    if (camera_) {
        camera_->release();
        camera_.reset();
    }

    if (cm_) {
        cm_->stop();
    }
} 

int main(){

  BallTracker camera("../../../config/params.yaml");
  
  camera.startCalibration();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  
  camera.stopCamera();
  std::this_thread::sleep_for(std::chrono::seconds(3));
  
  camera.startCamera();
  std::this_thread::sleep_for(std::chrono::seconds(3));

  camera.shutdown();

}

// Compile
//  mkdir -p build && g++ -std=c++17 ball_tracker.cpp -o build/ball_tracker `pkg-config --cflags --libs opencv4 libcamera` -lyaml-cpp
