#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <sys/mman.h>

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <yaml-cpp/yaml.h>

using namespace libcamera;
using namespace std::chrono_literals;

static std::shared_ptr<Camera> camera;

YAML::Node config = YAML::LoadFile("../../../config/params.yaml")["perception"]["camera"];

constexpr int MILLISECONDS_IN_SECOND = 1000000;

const int CAMERA_STREAM_INDEX = config["CAMERA_STREAM_INDEX"].as<int>();
const int CAMERA_STREAM_WIDTH = config["CAMERA_STREAM_WIDTH"].as<int>();
const int CAMERA_STREAM_HEIGHT = config["CAMERA_STREAM_HEIGHT"].as<int>();
const int CAMERA_FRAME_RATE = config["CAMERA_FRAME_RATE"].as<int>();
const int CAMERA_MIN_FRAME_DURATION = MILLISECONDS_IN_SECOND / CAMERA_FRAME_RATE;
const int CAMERA_MAX_FRAME_DURATION = MILLISECONDS_IN_SECOND / CAMERA_FRAME_RATE;
const bool CAMERA_AE_ENABLE = config["CAMERA_AE_ENABLE"].as<bool>();
const int CAMERA_AE_WARMUP_FRAMES = config["CAMERA_AE_WARMUP_FRAMES"].as<int>();
const int CAMERA_EXPOSURE_TIME = config["CAMERA_EXPOSURE_TIME"].as<int>();

const int IMAGE_DOWNSAMPLING_FACTOR = config["IMAGE_DOWNSAMPLING_FACTOR"].as<int>();
const int IMAGE_CROP_WIDTH = config["IMAGE_CROP_WIDTH"].as<int>();
const int IMAGE_CROP_HEIGHT = config["IMAGE_CROP_HEIGHT"].as<int>();

const float CALIBRATION_MARGIN = config["CALIBRATION_MARGIN"].as<float>();
const float CALIBRATION_X_PX = config["CALIBRATION_X_PX"].as<float>();
const float CALIBRATION_Y_PX = config["CALIBRATION_Y_PX"].as<float>();
const int CALIBRATION_RADIUS_PX = config["CALIBRATION_RADIUS_PX"].as<int>();

const int BALL_HUE_MAX = config["BALL_HUE_MAX"].as<int>();
const int BALL_HUE_MIN = config["BALL_HUE_MIN"].as<int>();
const int BALL_SAT_MIN = config["BALL_SAT_MIN"].as<int>();
const int BALL_SAT_MAX = config["BALL_SAT_MAX"].as<int>();
const int BALL_VALUE_MIN = config["BALL_VALUE_MIN"].as<int>();
const int BALL_VALUE_MAX = config["BALL_VALUE_MAX"].as<int>();

const int HSV_HUE_MIN = config["HSV_HUE_MIN"].as<int>();
const int HSV_SAT_MIN = config["HSV_SAT_MIN"].as<int>();
const int HSV_VALUE_MIN = config["HSV_VALUE_MIN"].as<int>();
const int HSV_HUE_MAX = config["HSV_HUE_MAX"].as<int>();
const int HSV_SAT_MAX = config["HSV_SAT_MAX"].as<int>();
const int HSV_VALUE_MAX = config["HSV_VALUE_MAX"].as<int>();

// TODO: delete
constexpr int FRAME_2_SAVE = 30;
constexpr const char* FRAME_NAME = "ball_hough.jpg";

// Colour Calibration
//   Default Values
int ball_hue_min = BALL_HUE_MIN;
int ball_hue_max = BALL_HUE_MAX;
int ball_sat_min = BALL_SAT_MIN;
int ball_sat_max = BALL_SAT_MAX;
int ball_value_min = BALL_VALUE_MIN;
int ball_value_max = BALL_VALUE_MAX;

// delete
bool calibrate_colour = true;

// Ball Location
int ball_x = -1;
int ball_y = -1;
int ball_radius = -1;
bool ball_values_stale = true;

cv::Mat stream_buffer_to_bgrx(uint8_t *&ptr,
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

void calibrate_ball_colour(FrameBuffer *buffer, StreamConfiguration const &cfg){

  uint8_t *frame_data;

  cv::Mat bgrxFrame = stream_buffer_to_bgrx(frame_data, buffer, cfg);

  cv::Mat bgrFrame;
  cv::cvtColor(bgrxFrame, bgrFrame, cv::COLOR_BGRA2BGR);                    
  
  cv::Mat hsvFrame;                                                             
  cv::cvtColor(bgrFrame, hsvFrame, cv::COLOR_BGR2HSV);          

  cv::Point2f center(CALIBRATION_X_PX, CALIBRATION_Y_PX);
  cv::Mat mask = cv::Mat::zeros(hsvFrame.size(), CV_8UC1);
  cv::circle(mask, center, CALIBRATION_RADIUS_PX, 255, -1);

  cv::Scalar mean, stddev;
  cv::meanStdDev(hsvFrame, mean, stddev, mask);

  int hue_min = mean[0] - CALIBRATION_MARGIN * stddev[0];
  int hue_max = mean[0] + CALIBRATION_MARGIN * stddev[0];
  int sat_min = mean[1] - CALIBRATION_MARGIN * stddev[1];
  int sat_max = mean[1] + CALIBRATION_MARGIN * stddev[1];
  int value_min = mean[2] - CALIBRATION_MARGIN * stddev[2];
  int value_max = mean[2] + CALIBRATION_MARGIN * stddev[2];

  ball_hue_min = std::clamp(hue_min, HSV_HUE_MIN, HSV_HUE_MAX);
  ball_hue_max = std::clamp(hue_max, HSV_HUE_MIN, HSV_HUE_MAX);
  ball_sat_min = std::clamp(sat_min, HSV_SAT_MIN, HSV_SAT_MAX);
  ball_sat_max = std::clamp(sat_max, HSV_SAT_MIN, HSV_SAT_MAX);
  ball_value_min = std::clamp(value_min, HSV_VALUE_MIN, HSV_VALUE_MAX);
  ball_value_max = std::clamp(value_max, HSV_VALUE_MIN, HSV_VALUE_MAX);

  std::cout << "HSV MIN: " << ball_hue_min << ", " 
                           << ball_sat_min << ", "
                           << ball_value_min << ", "
                           << std::endl;

  std::cout << "HSV MAX: " << ball_hue_max << ", "
                           << ball_sat_max << ", "
                           << ball_value_max << ", "
                           << std::endl;

  cv::Mat masked_result;
  bgrFrame.copyTo(masked_result, mask);
  cv::imwrite("masked_frame.jpg", masked_result);

  munmap(frame_data, buffer->planes()[0].length); 
}

void detectPingPongBall(cv::Mat &frame, cv::Mat &mask){

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
        
        ball_x = center.x;
        ball_y = center.y;
        ball_radius = radius;
        ball_values_stale = false;
        std::cout << "Ball at (" << center.x << ", " << center.y 
                  << ") radius=" << radius << std::endl;
        
        // Draw detection
        cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
        cv::circle(frame, center, 3, cv::Scalar(0, 0, 255), -1);
    }
    else{
      ball_values_stale = true;
    }
}

void frameProcessing(FrameBuffer *buffer, StreamConfiguration const &cfg, bool save_frame){      
  
  uint8_t *frame_data;
  cv::Mat bgrxFrame = stream_buffer_to_bgrx(frame_data, buffer, cfg);

  auto t2 = std::chrono::high_resolution_clock::now();  

  cv::Rect roi ((CAMERA_STREAM_WIDTH-CAMERA_STREAM_HEIGHT)/2, 
                 0, 
                 CAMERA_STREAM_HEIGHT, 
                 CAMERA_STREAM_HEIGHT);

  bgrxFrame =  bgrxFrame(roi);

  cv::resize(bgrxFrame, 
              bgrxFrame, 
              cv::Size(CAMERA_STREAM_HEIGHT/IMAGE_DOWNSAMPLING_FACTOR,
                      CAMERA_STREAM_HEIGHT/IMAGE_DOWNSAMPLING_FACTOR));  
     
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
  detectPingPongBall(bgrFrame, mask);                                                 
                                         
  auto t8 = std::chrono::high_resolution_clock::now();
     
  if(save_frame) cv::imwrite(FRAME_NAME, bgrFrame);
                                  
  munmap(frame_data, buffer->planes()[0].length);    
                                                                    
  std::cout << "Resize/Crop: " << std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count() << "μs\n";
  std::cout << "BGR: " << std::chrono::duration_cast<std::chrono::microseconds>(t4-t3).count() << "μs\n";
  std::cout << "HSV: " << std::chrono::duration_cast<std::chrono::microseconds>(t5-t4).count() << "μs\n";
  std::cout << "Mask: " << std::chrono::duration_cast<std::chrono::microseconds>(t6-t5).count() << "μs\n";
  std::cout << "Morph: " << std::chrono::duration_cast<std::chrono::microseconds>(t7-t6).count() << "μs\n";
  std::cout << "Detect: " << std::chrono::duration_cast<std::chrono::microseconds>(t8-t7).count() << "μs\n";
}  

static void requestComplete(Request *request)
{
  if (request->status() == Request::RequestCancelled)
    return;

  for (auto const &p : request->buffers()) {
    FrameBuffer *buffer = p.second;
		const StreamConfiguration &cfg = p.first->configuration();
    const FrameMetadata &meta = buffer->metadata();
    
    // TODO: make lighting calibration
    if(CAMERA_AE_ENABLE && meta.sequence <= CAMERA_AE_WARMUP_FRAMES){
      if(meta.sequence == CAMERA_AE_WARMUP_FRAMES){
        const ControlList &metadata = request->metadata();
        auto exposureOpt = metadata.get(controls::ExposureTime);
        int32_t exposureTime;

        if (exposureOpt.has_value()) {
          exposureTime = exposureOpt.value();
        }
        std::unique_ptr<libcamera::ControlList> lock_controls = 
          std::make_unique<libcamera::ControlList>();
        
        lock_controls->set(libcamera::controls::AeEnable, false);
        lock_controls->set(libcamera::controls::ExposureTime, exposureTime);
        request->controls() = std::move(*lock_controls);
      }
    }
    else if(calibrate_colour){
      calibrate_ball_colour(buffer, cfg);
      calibrate_colour = false;
    }
    else{
      frameProcessing(buffer, cfg, (meta.sequence == FRAME_2_SAVE));
    }

    std::cout << "Frame seq=" << meta.sequence << std::endl;
  }

  request->reuse(Request::ReuseBuffers);
  camera->queueRequest(request);
}

int main()
{

  auto cm = std::make_unique<CameraManager>();
  cm->start();

  auto cameras = cm->cameras();
  if (cameras.empty()) {
      std::cerr << "No cameras found\n";
      return 1;
  }

  camera = cm->get(cameras[0]->id());
  if (!camera) return 1;
  camera->acquire();

  auto config = camera->generateConfiguration({ StreamRole::Viewfinder });
  if (!config || config->size() != 1) return 1;

  StreamConfiguration &streamConfig = config->at(CAMERA_STREAM_INDEX);
  streamConfig.size.width = CAMERA_STREAM_WIDTH;
  streamConfig.size.height = CAMERA_STREAM_HEIGHT;
  config->validate();

  if (camera->configure(config.get()) < 0) return 1;

  auto *allocator = new FrameBufferAllocator(camera);
  if (allocator->allocate(streamConfig.stream()) < 0) return 1;

  Stream *stream = streamConfig.stream();
  const auto &buffers = allocator->buffers(stream);
  std::vector<std::unique_ptr<Request>> requests;

  for (const auto &buffer : buffers) {
      auto request = camera->createRequest();
      if (!request) return 1;
      if (request->addBuffer(stream, buffer.get()) < 0) return 1;
      requests.push_back(std::move(request));
  }

  camera->requestCompleted.connect(requestComplete);
  std::unique_ptr<libcamera::ControlList> controls = 
	  std::make_unique<libcamera::ControlList>();
  controls->set(libcamera::controls::FrameDurationLimits,
		  libcamera::Span<const int64_t, 2>(
		    {CAMERA_MIN_FRAME_DURATION,
		     CAMERA_MAX_FRAME_DURATION}));
  controls->set(libcamera::controls::AeEnable, CAMERA_AE_ENABLE);
  if(!CAMERA_AE_ENABLE)
    controls->set(libcamera::controls::ExposureTime, CAMERA_EXPOSURE_TIME);

  camera->start(controls.get());

  for (auto &request : requests)
      camera->queueRequest(request.get());

  std::this_thread::sleep_for(3s);

  cv::destroyAllWindows();

  camera->stop();
  allocator->free(stream);
  delete allocator;
  camera->release();
  camera.reset();
  cm->stop();
  return 0;
}

// compile
// g++ -std=c++17 camera.cpp -o camera `pkg-config --cflags --libs opencv4 libcamera` -lyaml-cpp
