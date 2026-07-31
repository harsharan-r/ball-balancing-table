// main_config.h
#pragma once

#include <cstdint>
#include <yaml-cpp/yaml.h>

struct MainConfig {
    int button_left_gpio_pin;
    int button_right_gpio_pin;
    int led_red_gpio_pin;
    int led_blue_gpio_pin;
    int led_green_gpio_pin;

    uint8_t pca9685_addr;
    int pca9685_freq;
    int servo_center_channel;
    int servo_left_channel;
    int servo_right_channel;

    int servo_center_min_pulse_us;
    int servo_center_max_pulse_us;
    int servo_left_min_pulse_us;
    int servo_left_max_pulse_us;
    int servo_right_min_pulse_us;
    int servo_right_max_pulse_us;

    int main_loop_delay_ms;

    MainConfig() = default;

    explicit MainConfig(YAML::Node config) {
        button_left_gpio_pin = config["BUTTON_LEFT_GPIO_PIN"].as<int>();
        button_right_gpio_pin = config["BUTTON_RIGHT_GPIO_PIN"].as<int>();
        led_red_gpio_pin = config["LED_RED_GPIO_PIN"].as<int>();
        led_blue_gpio_pin = config["LED_BLUE_GPIO_PIN"].as<int>();
        led_green_gpio_pin = config["LED_GREEN_GPIO_PIN"].as<int>();

        pca9685_addr = static_cast<uint8_t>(config["PCA9685_ADDR"].as<int>());
        pca9685_freq = config["PCA9685_FREQ"].as<int>();
        servo_center_channel = config["SERVO_CENTER_CHANNEL"].as<int>();
        servo_left_channel = config["SERVO_LEFT_CHANNEL"].as<int>();
        servo_right_channel = config["SERVO_RIGHT_CHANNEL"].as<int>();

        servo_center_min_pulse_us = config["SERVO_CENTER_MIN_PULSE_US"].as<int>();
        servo_center_max_pulse_us = config["SERVO_CENTER_MAX_PULSE_US"].as<int>();
        servo_left_min_pulse_us = config["SERVO_LEFT_MIN_PULSE_US"].as<int>();
        servo_left_max_pulse_us = config["SERVO_LEFT_MAX_PULSE_US"].as<int>();
        servo_right_min_pulse_us = config["SERVO_RIGHT_MIN_PULSE_US"].as<int>();
        servo_right_max_pulse_us = config["SERVO_RIGHT_MAX_PULSE_US"].as<int>();

        main_loop_delay_ms = config["MAIN_LOOP_DELAY_MS"].as<int>();
    }
};
