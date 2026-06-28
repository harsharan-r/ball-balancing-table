#include "../include/main.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <pigpio.h>

constexpr int button_left_GPIO_pin = 21;
constexpr int button_right_GPIO_pin = 19;
constexpr int led_red_GPIO_pin = 12;
constexpr int led_blue_GPIO_pin = 16;
constexpr int led_green_GPIO_pin = 13;

std::atomic<bool> keepRunning(true);

bool button_left_released = true;
bool button_right_released = true;

std::string state = "idle";

void setup_gpio(){
    gpioSetMode(led_red_GPIO_pin, PI_OUTPUT);
    gpioSetMode(led_blue_GPIO_pin, PI_OUTPUT);
    gpioSetMode(led_green_GPIO_pin, PI_OUTPUT);
    gpioSetMode(button_left_GPIO_pin, PI_INPUT);
    gpioSetMode(button_right_GPIO_pin, PI_INPUT);

    gpioSetPullUpDown(button_left_GPIO_pin, PI_PUD_UP);
    gpioSetPullUpDown(button_right_GPIO_pin, PI_PUD_UP);
    
    gpioWrite(led_red_GPIO_pin, PI_LOW);
    gpioWrite(led_blue_GPIO_pin, PI_LOW);
    gpioWrite(led_green_GPIO_pin, PI_LOW);
}

void set_led_colour(int red, int green, int blue){
    gpioPWM(led_red_GPIO_pin, red);
    gpioPWM(led_blue_GPIO_pin, blue);
    gpioPWM(led_green_GPIO_pin, green);
}

void cleanup_gpio(){
    gpioPWM(led_red_GPIO_pin, 0); 
    gpioPWM(led_blue_GPIO_pin, 0); 
    gpioPWM(led_green_GPIO_pin, 0); 

    // This stops the internal pigpio PWM peripheral threads safely.
    gpioSetMode(led_red_GPIO_pin, PI_INPUT);
    gpioSetMode(led_blue_GPIO_pin, PI_INPUT);
    gpioSetMode(led_green_GPIO_pin, PI_INPUT);
}

void signalHandler(int signum) {
    keepRunning = false;
}

int main(){
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed." << std::endl;
        gpioTerminate(); 
        return 1;
    }
    std::cout << "pigpio initialized successfully." << std::endl;

    gpioSetSignalFunc(2, signalHandler);
    setup_gpio();


    auto ball_x = std::make_shared<double>(0.0);
    auto ball_y = std::make_shared<double>(0.0);
    auto ball_radius = std::make_shared<double>(0.0);
    std::shared_mutex ball_mtx;

    BallTracker ball_tracker("../config/params.yaml", ball_x, ball_y, ball_radius, ball_mtx);

    while(keepRunning){

        if(state == "idle"){
            set_led_colour(250,35,0);
            if(gpioRead(button_left_GPIO_pin) == 0 && button_left_released) state = "ready";
        }
        else if(state == "ready"){
            set_led_colour(0,0,255);

            // Transition to idle state
            if(gpioRead(button_left_GPIO_pin) == 0 && gpioRead(button_right_GPIO_pin) == 0) {
                state = "idle";
            }
            // Transition to running state
            else if(gpioRead(button_left_GPIO_pin) == 0 && button_left_released){
                ball_tracker.startTracking();
                state = "running";
            } 
            // Transition to calibration state
            else if(gpioRead(button_right_GPIO_pin) == 0 && button_right_released){
                ball_tracker.startCalibration();
                state = "calibration";
            }
        }
        else if(state == "running"){
            set_led_colour(150,150,15);
            // Transition to ready state
            if(gpioRead(button_left_GPIO_pin) == 0 && button_left_released){
                ball_tracker.stopCamera();
                state = "ready";
            } 
        }
        else if(state == "calibration"){
            set_led_colour(100,0,180);
            std::shared_lock<std::shared_mutex> lock(ball_mtx);
            // Transition to ready state
            if(ball_tracker.is_calibrated) {
                ball_tracker.stopCamera();
                state = "ready";
            }
        }

        if(gpioRead(button_left_GPIO_pin) == 0 && button_left_released) button_left_released = false;
        else if(gpioRead(button_left_GPIO_pin) == 1 && !button_left_released) button_left_released = true;

        if(gpioRead(button_right_GPIO_pin) == 0 && button_right_released) button_right_released = false;
        else if(gpioRead(button_right_GPIO_pin) == 1 && !button_right_released) button_right_released = true;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ball_tracker.shutdown();
    cleanup_gpio();
    gpioTerminate();

    return 0;
}

//compile
// mkdir -p build &&  g++ -std=c++17 main.cpp ./perception/ball_tracker/ball_tracker.cpp -o build/main `pkg-config --cflags --libs opencv4 libcamera` -lyaml-cpp -lpigpio
// g++ -Wall -pthread -o pigpio_blink main.cpp -lpigpio -lyaml-cpp