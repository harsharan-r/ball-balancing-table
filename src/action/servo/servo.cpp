// servo.cpp
#include "../../../include/servo.h"
#include <wiringPiI2C.h>
#include <unistd.h>
#include <iostream>
#include <stdexcept>
#include <string>

// ── PCA9685 ──────────────────────────────────────────────
PCA9685::PCA9685(uint8_t addr, int freq) {
    fd = wiringPiI2CSetup(addr);
    if (fd < 0) {
        throw std::runtime_error("Failed to open I2C device at address 0x40");
    }

    reset();
    wiringPiI2CWriteReg8(fd, MODE2_REG, 0x04);
    setFreq(freq);
}

void PCA9685::reset() {
    wiringPiI2CWriteReg8(fd, MODE1_REG, 0x00);
    usleep(5000);
}

void PCA9685::setFreq(int freq) {
    int prescale = (int)(25000000.0f / (4096.0f * freq) + 0.5f) - 1;
    int oldMode  = wiringPiI2CReadReg8(fd, MODE1_REG);

    wiringPiI2CWriteReg8(fd, MODE1_REG,     (oldMode & 0x7F) | 0x10);
    wiringPiI2CWriteReg8(fd, PRE_SCALE_REG, prescale);
    wiringPiI2CWriteReg8(fd, MODE1_REG,     oldMode);
    usleep(5000);
    wiringPiI2CWriteReg8(fd, MODE1_REG,     oldMode | 0xA0);
}

void PCA9685::setPWM(int channel, int on, int off) {
    int base = LED0_BASE_REG + 4 * channel;
    wiringPiI2CWriteReg8(fd, base,     on  & 0xFF);
    wiringPiI2CWriteReg8(fd, base + 1, on  >> 8);
    wiringPiI2CWriteReg8(fd, base + 2, off & 0xFF);
    wiringPiI2CWriteReg8(fd, base + 3, off >> 8);
}

// ── Servo ─────────────────────────────────────────────────
Servo::Servo(PCA9685& board, int channel, int minPulse, int maxPulse)
    : brd(board),
      servo_channel(channel),
      SERVO_MIN((int)(minPulse / US_PER_TICK)),
      SERVO_MAX((int)(maxPulse / US_PER_TICK))
{}

int Servo::angleToPulse(float angle) {
    if (angle < MIN_ANGLE || angle > MAX_ANGLE) return -1;
    return (int)(SERVO_MIN + (angle / 180.0f) * (SERVO_MAX - SERVO_MIN));
}

void Servo::setPWM(int on, int off) {
    brd.setPWM(servo_channel, on, off);
}

void Servo::setAngle(float angle) {
    int pulse = angleToPulse(angle);
    if (pulse < 0)
        std::cerr << "Angle must be between " << MIN_ANGLE << " and " << MAX_ANGLE << std::endl;
    else
        setPWM(0, pulse);
}

// ── main ──────────────────────────────────────────────────
int main() {
    PCA9685 hat(0x40, 50);

    Servo servo_left  (hat,  9);
    Servo servo_right (hat, 10);
    Servo servo_center(hat, 11);

    servo_left.setAngle(170 - 40);
    servo_right.setAngle(170 - 40);
    servo_center.setAngle(170 - 40);
    sleep(1);
    return 0;
}
