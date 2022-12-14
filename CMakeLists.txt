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
    message_generation
)

# Service files.
add_service_files(
    FILES
    LidarScan.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

###########
## Build ##
###########
# Arduino config.
rosserial_generate_ros_lib(
    PACKAGE rosserial_arduino
    SCRIPT make_libraries.py
)

add_dependencies(${PROJECT_NAME}_ros_lib ${${PROJECT_NAME}_EXPORTED_TARGETS})

rosserial_configure_client(
    DIRECTORY firmware
    TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware firmware ALL)
rosserial_add_client_target(firmware firmware-upload)
