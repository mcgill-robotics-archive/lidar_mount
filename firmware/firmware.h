/*
 * Reading of encoder and publishing to ROS.
 */

#ifndef LIDAR_MOUNT_FIRMWARE
#define LIDAR_MOUNT_FIRMWARE

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <Arduino.h>
#include <Servo.h>

#include "AbsoluteEncoder.h"
#include "LidarTfBroadcaster.h"

// from math.h
# define M_PI 3.14159265358979323846  /* pi */

/*
 * Servo position intermediate method.
 */
void enable_scan (const std_msgs::Bool& scan);

#endif // LIDAR_MOUNT_FIRMWARE
