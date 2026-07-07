cmake_minimum_required(VERSION 3.16)
project(BallBalancingRobot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)

pkg_check_modules(OPENCV REQUIRED IMPORTED_TARGET opencv4)
pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

add_library(core
    src/action/servo/servo.cpp
    src/control/balance_controller/balance_controller.cpp
    src/control/pid_controller/pid_controller.cpp
    src/perception/ball_tracker/ball_tracker.cpp
)

target_compile_definitions(core
    PUBLIC
        APP_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/params.yaml"
)

target_include_directories(core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(core
    PUBLIC
        yaml-cpp
        pigpio
	PkgConfig::LIBCAMERA
        PkgConfig::OPENCV
	wiringPi
)

add_executable(main src/main.cpp)
target_link_libraries(main PRIVATE core)

# cmake -B build
# cmake --build build
