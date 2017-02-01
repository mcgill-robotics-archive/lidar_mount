/*
 * Reading of encoder and publishing to ROS.
 */

#ifndef LIDAR_MOUNT_FIRMWARE
#define LIDAR_MOUNT_FIRMWARE

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <Arduino.h>
#include <Servo.h>

#include "AbsoluteEncoder.h"

// from math.h
# define M_PI 3.14159265358979323846  /* pi */

/*
 * Servo position intermediate method.
 */
void enable_scan (const std_msgs::Bool& scan);

#endif // LIDAR_MOUNT_FIRMWARE
