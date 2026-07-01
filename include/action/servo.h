// servo.h
#pragma once

#include <cstdint>
#include <stdexcept>

class PCA9685 {
public:
    // Opens I2C device at addr, resets chip, sets frequency
    // Throws std::runtime_error if I2C device fails to open
    PCA9685(uint8_t addr = 0x40, int freq = 50);

    void reset();
    void setFreq(int freq);
    void setPWM(int channel, int on, int off);

private:
    int fd;

    static const uint8_t MODE1_REG     = 0x00;
    static const uint8_t MODE2_REG     = 0x01;
    static const uint8_t PRE_SCALE_REG = 0xFE;
    static const uint8_t LED0_BASE_REG = 0x06;
};


class Servo {
public:
    // board    — reference to an already-initialized PCA9685
    // channel  — which of the 16 channels this servo is on (0–15)
    // minPulse — pulse width in µs that corresponds to 0 deg (default 500)
    // maxPulse — pulse width in µs that corresponds to 180 deg(default 2700)
    Servo(PCA9685& board,
          int channel,
          int minPulse = 500,
          int maxPulse = 2700);

    // Writes the angle to the chip. Prints error if out of range.
    void setAngle(float angle);

private:
    void setPWM(int on, int off);
    int  angleToPulse(float angle);

    // Register base address for channel calculation
    static const uint8_t LED0_BASE_REG = 0x06;

    // PWM cycle constants
    static constexpr float CYCLE_LENGTH    = 20000.0f; // µs (1/50Hz)
    static constexpr float CYCLE_DIVISIONS = 4096.0f;  // 12-bit counter
    static constexpr float US_PER_TICK     = CYCLE_LENGTH / CYCLE_DIVISIONS; // ~4.88µs

    // Angle limits
    static constexpr float MIN_ANGLE = 0.0f;
    static constexpr float MAX_ANGLE = 180.0f;

    PCA9685& brd;       // reference to shared board — never copied
    int servo_channel;
    int SERVO_MIN;      // min pulse in ticks, derived from minPulse µs
    int SERVO_MAX;      // max pulse in ticks, derived from maxPulse µs
};
