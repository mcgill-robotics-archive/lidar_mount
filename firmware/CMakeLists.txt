cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

include_directories(${ARDUINO_SDK}/libraries)
include_directories(${ARDUINO_SDK}/hardware/arduino/avr/libraries)

generate_arduino_firmware(firmware
    SRCS firmware.cpp firmware.h ${ROS_LIB_DIR}/time.cpp
    BOARD uno
    PORT /dev/ttyUSB-1
)
