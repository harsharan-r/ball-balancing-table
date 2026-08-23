#include "perception/ball_tracker/ball_tracker.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <chrono>
#include <shared_mutex>

#include <sys/mman.h>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

using namespace libcamera;


BallTracker::BallTracker(const std::string& config_path, 
                         std::shared_ptr<double> ball_x, 
                         std::shared_ptr<double> ball_y, 
                         std::shared_ptr<double> ball_radius,
                         std::shared_ptr<bool> ball_stale,
                         std::shared_mutex& ball_mtx,
                         std::shared_ptr<int> ball_capture_frame)
    : cm_(std::make_unique<CameraManager>()),
      ball_x_(ball_x),
      ball_y_(ball_y),
      ball_radius_(ball_radius),
      ball_stale_(ball_stale),
      ball_mtx_(ball_mtx),
      ball_capture_frame_(ball_capture_frame)
{
    YAML::Node config = YAML::LoadFile(config_path)["perception"]["ball_tracker"];
    cfg_ = BallTrackerConfig(config);

    ball_hue_min = cfg_.ball_search_hue_min;
    ball_hue_max = cfg_.ball_search_hue_max;
    ball_sat_min = cfg_.ball_search_sat_min;
    ball_sat_max = cfg_.ball_search_sat_max;
    ball_value_min = cfg_.ball_search_value_min;
    ball_value_max = cfg_.ball_search_value_max;

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

  controls_->set(controls::AeEnable, false);
  controls_->set(controls::ExposureTime, cfg_.camera_exposure_time);
  controls_->set(controls::AnalogueGain, cfg_.camera_analogue_gain);
  controls_->set(controls::AfMode, controls::AfModeManual);
  controls_->set(controls::LensPosition, cfg_.camera_lens_position);
}

void BallTracker::requestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled) {
        return;
    }

    auto now = std::chrono::high_resolution_clock::now();

    if (prev_frame_time_.has_value()) {
        double dt = std::chrono::duration<double>(now - prev_frame_time_.value()).count();
        if (dt > 0.0) {
            current_fps_ = 1.0 / dt;
            std::cout << "Camera FPS: " << current_fps_.value() << std::endl;
        }
    }

    prev_frame_time_ = now;

    ControlList pending_controls;

    for (const auto &p : request->buffers()) {
        FrameBuffer *buffer = p.second;
        const StreamConfiguration &cfg = p.first->configuration();
        const FrameMetadata &meta = buffer->metadata();

        (void)buffer;
        (void)cfg;

        if (mode == Mode::Calibration) {
            calibrate(request, buffer, cfg, pending_controls);
        }
        else if (mode == Mode::Track) {
            track(buffer, cfg);
        }
    }

    if (!is_camera_started) {
        return;
    }

    // request->reuse() clears the request's ControlList, so anything calibrate()
    // set on request->controls() would be wiped before queueing -- controls staged
    // in pending_controls survive because they're merged back in after reuse().
    request->reuse(Request::ReuseBuffers);
    request->controls().merge(pending_controls, ControlList::MergePolicy::OverwriteExisting);

    if (camera_->queueRequest(request) < 0) {
        throw std::runtime_error("Failed to queue request");
    }
}

void BallTracker::track(FrameBuffer *buffer, StreamConfiguration const &cfg, bool save_frame){

  uint8_t *frame_data;
  cv::Mat bgrxFrame = stream_buffer_to_bgrx(frame_data, buffer, cfg);

  auto t2 = std::chrono::high_resolution_clock::now();  

  int frame_center_x = (cfg_.camera_stream_width  - cfg_.image_crop_width)  / 2;
  int frame_center_y = (cfg_.camera_stream_height - cfg_.image_crop_height) / 2;

  cv::Rect roi(frame_center_x, frame_center_y, cfg_.image_crop_width, cfg_.image_crop_height);

  bgrxFrame =  bgrxFrame(roi);

  cv::resize(bgrxFrame, 
              bgrxFrame, 
              cv::Size(cfg_.image_crop_width/cfg_.image_downsampling_factor,
                      cfg_.image_crop_height/cfg_.image_downsampling_factor));  
     
  auto t3 = std::chrono::high_resolution_clock::now();
                 
  cv::Mat bgrFrame;
  
  cv::cvtColor(bgrxFrame, bgrFrame, cv::COLOR_BGRA2BGR);                                          
  
  auto t4 = std::chrono::high_resolution_clock::now();

  cv::Mat hsvFrame;
  cv::cvtColor(bgrFrame, hsvFrame, cv::COLOR_BGR2HSV);                     

  auto t5 = std::chrono::high_resolution_clock::now();

  cv::Scalar lower(ball_hue_min, ball_sat_min, ball_value_min);
  cv::Scalar upper(ball_hue_max, ball_sat_max, ball_value_max);

  cv::Mat mask;
  cv::inRange(hsvFrame, lower, upper, mask);
  
  auto t6 = std::chrono::high_resolution_clock::now();

  cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1),2);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1),2);

  auto t7 = std::chrono::high_resolution_clock::now();
  detectPingPongBall(bgrFrame, mask, buffer->metadata().sequence);                                                 
                                         
  auto t8 = std::chrono::high_resolution_clock::now();
     
  const FrameMetadata &meta = buffer->metadata();
  std::string image_name = "./images/ball_detection_" + std::to_string(meta.sequence) + ".jpg";
  if(save_frame) cv::imwrite(image_name, bgrFrame);
                                  
  munmap(frame_data, buffer->planes()[0].length);    
  
  // TODO: remove and remove timing variables
  // std::cout << "Resize/Crop: " << std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count() << "μs\n";
  // std::cout << "BGR: " << std::chrono::duration_cast<std::chrono::microseconds>(t4-t3).count() << "μs\n";
  // std::cout << "HSV: " << std::chrono::duration_cast<std::chrono::microseconds>(t5-t4).count() << "μs\n";
  // std::cout << "Mask: " << std::chrono::duration_cast<std::chrono::microseconds>(t6-t5).count() << "μs\n";
  // std::cout << "Morph: " << std::chrono::duration_cast<std::chrono::microseconds>(t7-t6).count() << "μs\n";
  // std::cout << "Detect: " << std::chrono::duration_cast<std::chrono::microseconds>(t8-t7).count() << "μs\n";
}

void BallTracker::detectPingPongBall(cv::Mat &frame, cv::Mat &mask, int frame_number){

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  if (!contours.empty()) {
    auto largest = std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });
    
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(*largest, center, radius);
    

    std::unique_lock<std::shared_mutex> lock(ball_mtx_);

    double half_crop_width = cfg_.image_crop_width/cfg_.image_downsampling_factor/2;
    double half_crop_height = cfg_.image_crop_height/cfg_.image_downsampling_factor/2;

    *ball_x_ = (center.x-half_crop_width)/half_crop_width;
    *ball_y_ = (center.y-half_crop_height)/half_crop_height;
    *ball_radius_ = static_cast<double>(radius)/(half_crop_width+half_crop_width);
    *ball_stale_ = false;
    *ball_capture_frame_ = frame_number;

    std::cout << "Ball at (" << *ball_x_ << ", " << *ball_y_
              << ") radius=" << *ball_radius_ << std::endl;

    lock.unlock();
    
    // Draw detection
    cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
    cv::circle(frame, center, 3, cv::Scalar(0, 0, 255), -1);
  }
  else{
    std::unique_lock<std::shared_mutex> lock(ball_mtx_);
    *ball_stale_ = true;
    lock.unlock();
  }
}

void BallTracker::calibrate(libcamera::Request *request, libcamera::FrameBuffer *buffer, StreamConfiguration const &cfg, libcamera::ControlList &pending_controls){
  std::shared_lock<std::shared_mutex> shr_lock(ball_mtx_);
  if (is_calibrated) {
    return;
  }
  shr_lock.unlock();

  if (calibration_frames == 0) {
    pending_controls.set(libcamera::controls::AeEnable, true);
    pending_controls.set(libcamera::controls::AfMode, libcamera::controls::AfModeAuto);
    pending_controls.set(libcamera::controls::AfTrigger, libcamera::controls::AfTriggerStart);
    std::cout << "Re-enabling AE and triggering AF for calibration" << std::endl;
    calibration_frames++;
    return;
  }

  if (!af_locked_) {
    const ControlList &metadata = request->metadata();
    auto af_state_opt = metadata.get(libcamera::controls::AfState);
    bool af_converged = af_state_opt.has_value() &&
        (af_state_opt.value() == libcamera::controls::AfStateFocused ||
         af_state_opt.value() == libcamera::controls::AfStateFailed);

    calibration_frames++;
    if (!af_converged && calibration_frames < cfg_.camera_af_warmup_frames) {
      std::cout << "Waiting for autofocus to converge, frame " << calibration_frames << std::endl;
      return;
    }

    if (af_state_opt.has_value() && af_state_opt.value() == libcamera::controls::AfStateFailed) {
      std::cout << "Autofocus failed to converge, locking best-effort lens position" << std::endl;
    } else if (!af_converged) {
      std::cout << "Autofocus warmup frame budget exhausted, locking best-effort lens position" << std::endl;
    }

    auto lens_position_opt = metadata.get(libcamera::controls::LensPosition);
    if (!lens_position_opt.has_value()) {
        throw std::runtime_error("LensPosition not found in metadata");
    }
    float lens_position = lens_position_opt.value();
    locked_lens_position_ = lens_position;

    pending_controls.set(libcamera::controls::AfMode, libcamera::controls::AfModeManual);
    pending_controls.set(libcamera::controls::LensPosition, lens_position);
    controls_->set(libcamera::controls::AfMode, libcamera::controls::AfModeManual);
    controls_->set(libcamera::controls::LensPosition, lens_position);
    af_locked_ = true;
    calibration_frames = 0;
    std::cout << "Locking focus" << std::endl;
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
    auto gain_opt = metadata.get(libcamera::controls::AnalogueGain);

    if (!exposure_opt.has_value()) {
        throw std::runtime_error("ExposureTime not found in metadata");
    }
    if (!gain_opt.has_value()) {
        throw std::runtime_error("AnalogueGain not found in metadata");
    }

    int32_t exposure_time = exposure_opt.value();
    float analogue_gain = gain_opt.value();
    locked_exposure_time_ = exposure_time;
    locked_analogue_gain_ = analogue_gain;

    pending_controls.set(libcamera::controls::AeEnable, false);
    pending_controls.set(libcamera::controls::ExposureTime, exposure_time);
    pending_controls.set(libcamera::controls::AnalogueGain, analogue_gain);
    controls_->set(libcamera::controls::AeEnable, false);
    controls_->set(libcamera::controls::ExposureTime, exposure_time);
    controls_->set(libcamera::controls::AnalogueGain, analogue_gain);

    calibration_frames++;
    std::cout << "Locking exposure and gain" << std::endl;
    return;
  }

  calibrate_ball_colour(buffer, cfg);
  std::cout << "Calibrated colour, exposure time: " << locked_exposure_time_
             << ", analogue gain: " << locked_analogue_gain_
             << ", lens position: " << locked_lens_position_ << std::endl;

  std::unique_lock<std::shared_mutex> lock(ball_mtx_);
  is_calibrated = true;
  calibration_frames = 0;
  af_locked_ = false;
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

  ball_hue_min = std::clamp(hue_min, cfg_.hsv_clamp_hue_min, cfg_.hsv_clamp_hue_max);
  ball_hue_max = std::clamp(hue_max, cfg_.hsv_clamp_hue_min, cfg_.hsv_clamp_hue_max);
  ball_sat_min = std::clamp(sat_min, cfg_.hsv_clamp_sat_min, cfg_.hsv_clamp_sat_max);
  ball_sat_max = std::clamp(sat_max, cfg_.hsv_clamp_sat_min, cfg_.hsv_clamp_sat_max);
  ball_value_min = std::clamp(value_min, cfg_.hsv_clamp_value_min, cfg_.hsv_clamp_value_max);
  ball_value_max = std::clamp(value_max, cfg_.hsv_clamp_value_min, cfg_.hsv_clamp_value_max);

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
    mode = Mode::Idle;
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

// int main(){

//   auto ball_x = std::make_shared<double>(-1.0);
//   auto ball_y = std::make_shared<double>(-1.0);
//   auto ball_radius = std::make_shared<double>(-1.0);
//   std::shared_mutex ball_mtx;

//   BallTracker camera("../../../config/params.yaml", ball_x, ball_y, ball_radius, ball_mtx);
  
//   camera.startCalibration();
//   std::this_thread::sleep_for(std::chrono::seconds(3));
  
//   camera.stopCamera();
//   std::this_thread::sleep_for(std::chrono::seconds(3));
  
//   camera.startTracking();
//   std::this_thread::sleep_for(std::chrono::seconds(3));

//   camera.shutdown();

// }

// Compile
//  mkdir -p build && g++ -std=c++17 ball_tracker.cpp -o build/ball_tracker `pkg-config --cflags --libs opencv4 libcamera` -lyaml-cpp
