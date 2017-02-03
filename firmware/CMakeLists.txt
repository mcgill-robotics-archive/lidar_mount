cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(firmware
    SRCS firmware.cpp firmware.h ${ROS_LIB_DIR}/time.cpp
    BOARD uno
    PORT /dev/redBoard
)
