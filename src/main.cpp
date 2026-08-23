#include "main.h"
#include "main_config.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <memory>
#include <pigpio.h>
#include <yaml-cpp/yaml.h>

std::atomic<bool> keepRunning(true);

void setup_gpio(const MainConfig& cfg){
    gpioSetMode(cfg.led_red_gpio_pin, PI_OUTPUT);
    gpioSetMode(cfg.led_blue_gpio_pin, PI_OUTPUT);
    gpioSetMode(cfg.led_green_gpio_pin, PI_OUTPUT);
    gpioSetMode(cfg.button_left_gpio_pin, PI_INPUT);
    gpioSetMode(cfg.button_right_gpio_pin, PI_INPUT);

    gpioSetPullUpDown(cfg.button_left_gpio_pin, PI_PUD_UP);
    gpioSetPullUpDown(cfg.button_right_gpio_pin, PI_PUD_UP);

    gpioWrite(cfg.led_red_gpio_pin, PI_LOW);
    gpioWrite(cfg.led_blue_gpio_pin, PI_LOW);
    gpioWrite(cfg.led_green_gpio_pin, PI_LOW);
}

void set_led_colour(const MainConfig& cfg, int red, int green, int blue){
    gpioPWM(cfg.led_red_gpio_pin, red);
    gpioPWM(cfg.led_blue_gpio_pin, blue);
    gpioPWM(cfg.led_green_gpio_pin, green);
}

void cleanup_gpio(const MainConfig& cfg){
    gpioPWM(cfg.led_red_gpio_pin, 0);
    gpioPWM(cfg.led_blue_gpio_pin, 0);
    gpioPWM(cfg.led_green_gpio_pin, 0);

    // This stops the internal pigpio PWM peripheral threads safely.
    gpioSetMode(cfg.led_red_gpio_pin, PI_INPUT);
    gpioSetMode(cfg.led_blue_gpio_pin, PI_INPUT);
    gpioSetMode(cfg.led_green_gpio_pin, PI_INPUT);
}

void signalHandler(int signum) {
    keepRunning = false;
}

int main(){

    // CONFIG SETUP
    MainConfig cfg(YAML::LoadFile(APP_CONFIG_PATH)["main"]);

    // GPIO SETUP
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed." << std::endl;
        gpioTerminate();
        return 1;
    }
    std::cout << "pigpio initialized successfully." << std::endl;

    gpioSetSignalFunc(2, signalHandler);
    setup_gpio(cfg);

    // SERVO SETUP
    PCA9685 hat(cfg.pca9685_addr, cfg.pca9685_freq);
    Servo servo_center(hat, cfg.servo_center_channel, cfg.servo_center_min_pulse_us, cfg.servo_center_max_pulse_us);
    Servo servo_left  (hat, cfg.servo_left_channel,   cfg.servo_left_min_pulse_us,   cfg.servo_left_max_pulse_us);
    Servo servo_right (hat, cfg.servo_right_channel,  cfg.servo_right_min_pulse_us,  cfg.servo_right_max_pulse_us);

    // SHARED VALUES SETUP
    auto ball_x = std::make_shared<double>(0.0);
    auto ball_y = std::make_shared<double>(0.0);
    auto ball_radius = std::make_shared<double>(0.0);
    auto ball_stale_ = std::make_shared<bool>(true);
    auto ball_capture_frame_ = std::make_shared<int>(-1);
    std::shared_mutex ball_mtx_;

    // HELPER OBJECTS
    BallTracker ball_tracker(APP_CONFIG_PATH, ball_x, ball_y, ball_radius, ball_stale_, ball_mtx_, ball_capture_frame_);
    BalanceController balance_controller(APP_CONFIG_PATH, servo_center, servo_left, servo_right, ball_x, ball_y, ball_radius, ball_stale_, ball_mtx_, ball_capture_frame_);

    // STATE VARIABLES
    bool button_left_released = true;
    bool button_right_released = true;

    std::string state = "ready";

    std::chrono::milliseconds delay(cfg.main_loop_delay_ms);

    while(keepRunning){

        if(state == "idle"){
            set_led_colour(cfg, 250,35,0);

            balance_controller.idle();

            // Transition to ready state
            if(gpioRead(cfg.button_left_gpio_pin) == 0 && button_left_released) {
                // ==================== TEMP TUNING INSTRUMENTATION ====================
                balance_controller.reload_config();
                // =======================================================================
                state = "ready";
            }
        }
        else if(state == "ready"){
            set_led_colour(cfg, 0,0,255);

            balance_controller.ready();

            // Transition to idle state
            if(gpioRead(cfg.button_left_gpio_pin) == 0 && gpioRead(cfg.button_right_gpio_pin) == 0) {
                state = "idle";
            }
            // Transition to running state
            else if(gpioRead(cfg.button_left_gpio_pin) == 0 && button_left_released){
                ball_tracker.startTracking();
                balance_controller.start_balancing_thread();
                set_led_colour(cfg, 150,150,15);
                state = "running";
            }
            // Transition to calibration state
            else if(gpioRead(cfg.button_right_gpio_pin) == 0 && button_right_released){
                ball_tracker.startCalibration();
                state = "calibration";
            }
        }
        else if(state == "running"){

            // balance_controller.running();

            // Transition to ready state
            if(gpioRead(cfg.button_left_gpio_pin) == 0 && button_left_released){
                ball_tracker.stopCamera();
                balance_controller.stop_balancing_thread();
                // ==================== TEMP TUNING INSTRUMENTATION ====================
                balance_controller.plot_and_clear_log();
                balance_controller.reload_config();
                // =======================================================================
                state = "ready";
            }
        }
        else if(state == "calibration"){
            set_led_colour(cfg, 100,0,180);
            std::shared_lock<std::shared_mutex> lock(ball_mtx_);
            // Transition to ready state
            if(ball_tracker.is_calibrated) {
                ball_tracker.stopCamera();
                // ==================== TEMP TUNING INSTRUMENTATION ====================
                balance_controller.reload_config();
                // =======================================================================
                state = "ready";
            }
        }

        if(gpioRead(cfg.button_left_gpio_pin) == 0 && button_left_released) button_left_released = false;
        else if(gpioRead(cfg.button_left_gpio_pin) == 1 && !button_left_released) button_left_released = true;

        if(gpioRead(cfg.button_right_gpio_pin) == 0 && button_right_released) button_right_released = false;
        else if(gpioRead(cfg.button_right_gpio_pin) == 1 && !button_right_released) button_right_released = true;

        std::this_thread::sleep_for(delay);
    }

    ball_tracker.shutdown();
    cleanup_gpio(cfg);
    gpioTerminate();

    return 0;
}

//compile
// mkdir -p build &&  g++ -std=c++17 main.cpp ./perception/ball_tracker/ball_tracker.cpp ./action/servo/servo.cpp ./control/balance_controller/balance_controller.cpp ./control/pid_controller/pid_controller.cpp -o build/main `pkg-config --cflags --libs opencv4 libcamera` -lyaml-cpp -lpigpio -lwiringPi
// g++ -Wall -pthread -o pigpio_blink main.cpp -lpigpio -lyaml-cpp
// g++ servo.cpp -o servo -lwiringPi
