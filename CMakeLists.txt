cmake_minimum_required(VERSION 2.8.3)
project(lidar_mount)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    tf2
    tf2_ros
    rospy
    roscpp
    std_msgs
    geometry_msgs
    rosserial_arduino
    rosserial_client
)

catkin_package()

###########
## Build ##
###########

rosserial_generate_ros_lib(
    PACKAGE rosserial_arduino
    SCRIPT make_libraries.py
)

rosserial_configure_client(
    DIRECTORY firmware
    TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware firmware ALL)
rosserial_add_client_target(firmware firmware-upload)
