/*
 * Handle differential position control of the arm to a particular position.
 *
 * Author: Malcolm Watt
 */

#ifndef LIDAR_CONTROLLER
#define LIDAR_CONTROLLER

#include <ros.h>
#include <std_msgs/Int32.h>
#include <Arduino.h>
#include <Servo.h>

const int SERVO_PIN = 6;

class LidarController
{
private:
    int threshold;
public:
    int goal_angle;
    float current_encoder_angle;
    Servo servo;

    LidarController();
    
    /*
     * Track position based on internally stored goal_angle.
     */
    int track_position ();
};

#endif // LIDAR_CONTROLLER
