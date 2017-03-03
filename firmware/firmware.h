/*
 * Reading of encoder and publishing to ROS.
 */

#ifndef LIDAR_MOUNT_FIRMWARE
#define LIDAR_MOUNT_FIRMWARE

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <lidar_mount/LidarScan.h>

#include <Arduino.h>
#include <Servo.h>

#include "AbsoluteEncoder.h"
#include "ScanController.h"

/*
 * Servo position intermediate method.
 */
void enable_scan (const std_msgs::Bool& scan);

#endif // LIDAR_MOUNT_FIRMWARE
