# Ball Balancing Table 

A small deskstop ping pong ball balancing machine that uses a Pi Camera Module 3 to track the ping pong ball, and uses three MG 996R on a 3 DOF Stewart type platform. Everything is done via Raspberry Pi 3B+ 1 GB ram with a adafruit servo hat. The goal was to explore computer vision, state estimation, and motion controls. 

## Demo


## About this project

Three servo-actuated legs drive an RRS parallel linkage under a 120mm platform. A Pi Camera
tracks the ball's position via HSV color masking, a Kalman filter smooths that into a
position + velocity estimate, and independent PID loops on roll/pitch/height keep the ball
centered.

## Getting Started

### Prerequisites

- Raspberry Pi (tested on a Pi 3B+) running Raspberry Pi OS, with a Pi Camera Module and an
  Adafruit PCA9685 servo hat wired over I2C
- `cmake` >= 3.16, a C++17 compiler
- System packages: `opencv4`, `libcamera`, `yaml-cpp`, `pigpio`, `wiringPi`

### Installation

```
./balance setup   # installs system packages, enables I2C, configures the CMake build dir
./balance build   # cmake --build build
```

### Usage

```
./balance run
```

Runs `sudo ./build/main`, since GPIO/I2C access requires root. Use the two GPIO buttons to
move between idle / ready / calibration / running states (see `CLAUDE.md` for details).

`./balance` is a thin wrapper around `scripts/setup.sh`, `scripts/build.sh`, and
`scripts/run.sh`, which can also be run directly.
