#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <sys/mman.h>

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


using namespace libcamera;
using namespace std::chrono_literals;

static std::shared_ptr<Camera> camera;

constexpr int CAMERA_STREAM_INDEX = 0;
constexpr int CAMERA_STREAM_WIDTH = 2034;
constexpr int CAMERA_STREAM_HEIGHT = 1296;
constexpr int CAMERA_MIN_FRAME_DURATION = 8333;
constexpr int CAMERA_MAX_FRAME_DURATION = 8333;
constexpr bool CAMERA_AE_ENABLE = false;
constexpr int CAMERA_EXPOSURE_TIME = 6000;

constexpr int PROCESSING_IMAGE_WIDTH = 240;
constexpr int PROCESSING_IMAGE_HEIGHT = 135;
constexpr int BALL_HUE_MIN = 0;
constexpr int BALL_HUE_MAX = 32;
constexpr int BALL_SAT_MIN = 177;
constexpr int BALL_SAT_MAX = 255;
constexpr int BALL_VALUE_MIN = 101;
constexpr int BALL_VALUE_MAX = 255;

constexpr int FRAME_2_SAVE = 30;
constexpr const char* FRAME_NAME = "ball.jpg";

void detectPingPongBall(cv::Mat &frame, cv::Mat &mask){

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // Find largest contour (likely the ball)
    if (!contours.empty()) {
        auto largest = std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });
        
        // Get bounding circle
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(*largest, center, radius);
        
        // Filter by size (adjust min/max radius for your setup)
        if (true) {
            std::cout << "Ball at (" << center.x << ", " << center.y 
                      << ") radius=" << radius << std::endl;
            
            // Draw detectioni
            cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
            cv::circle(frame, center, 3, cv::Scalar(0, 0, 255), -1);
        }
    }
}

void frameProcessing(FrameBuffer *buffer, StreamConfiguration const &cfg, bool save_frame){      
  int fd = buffer->planes()[0].fd.get();                                        
  uint8_t *ptr = static_cast<uint8_t *>(                                        
    mmap(NULL,                                                                  
         buffer->planes()[0].length,                                            
         PROT_READ | PROT_WRITE,                                                
         MAP_SHARED,                                                            
         fd,                                                                    
         0)                                                                     
  );                                                                            
                                                                                
  cv::Mat bgrxFrame(cfg.size.height, cfg.size.width, CV_8UC4, ptr, cfg.stride);
  cv::resize(bgrxFrame, bgrxFrame, cv::Size(PROCESSING_IMAGE_WIDTH, PROCESSING_IMAGE_HEIGHT));  
                 
  cv::Mat bgrFrame;
  
  cv::cvtColor(bgrxFrame, bgrFrame, cv::COLOR_BGRA2BGR);                                          
  
  cv::Mat hsvFrame;
  cv::cvtColor(bgrFrame, hsvFrame, cv::COLOR_BGR2HSV);                     

  cv::Scalar lower(BALL_HUE_MIN, BALL_SAT_MIN, BALL_VALUE_MIN);
  cv::Scalar upper(BALL_HUE_MAX, BALL_SAT_MAX, BALL_VALUE_MAX);

  cv::Mat mask;
  cv::inRange(hsvFrame, lower, upper, mask);

  cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1),2);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1),2);

  detectPingPongBall(bgrFrame, mask);                                                 
                                              
  if(save_frame) cv::imwrite(FRAME_NAME, bgrFrame);
                                  
  munmap(ptr, buffer->planes()[0].length);                                                                                                                      
}  

static void requestComplete(Request *request)
{
  if (request->status() == Request::RequestCancelled)
    return;

  for (auto const &p : request->buffers()) {
    FrameBuffer *buffer = p.second;
		const StreamConfiguration &cfg = p.first->configuration();
    const FrameMetadata &meta = buffer->metadata();
		frameProcessing(buffer, cfg, (meta.sequence == FRAME_2_SAVE));
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
