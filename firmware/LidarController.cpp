/*
 * Handle differential position control of the arm to a particular position.
 *
 * Author: Malcolm Watt
 */
#include "LidarController.h"

LidarController::LidarController()
{
    goal_angle = 0;
    threshold = 1;
    current_encoder_angle = 0;

    servo.attach(SERVO_PIN);
}

int LidarController::track_position()
{
    int pot_value = servo.read();

    if (abs(current_encoder_angle - goal_angle) > threshold)
    {
        pot_value += (int) goal_angle - current_encoder_angle;
    }
    
    servo.write(pot_value);
    return pot_value;
}
