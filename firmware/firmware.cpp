/*
 * Firmware implementation for Lidar Mount.
 *
 * Author: Malcolm Watt
 */
#include "firmware.h"

// Arduino pins.
const int SERVO_PIN = 6;
const int CHIP_SELECT_PIN = 10;


Servo servo;
bool scan_flag = false;
bool scan_dir_up = true;
int servo_speed = 10;
int pitch_up_lim = 330;
int pitch_down_lim = 45;

ros::NodeHandle nh;

std_msgs::String test;
ros::Subscriber<std_msgs::Bool> sub_enable_scan("/lidar_mount/enable_scan",
    &enable_scan);

ros::Publisher test_pub("/lidar_mount/test", &test);

AbsoluteEncoder * encoder;
LidarTfBroadcaster * broadcaster;

void setup()
{
  // Setup the ROS node.
  nh.initNode();
  nh.subscribe(sub_enable_scan);
  nh.advertise(test_pub);

  // Test
  SPI.begin();

  encoder = new AbsoluteEncoder(CHIP_SELECT_PIN);
  broadcaster = new LidarTfBroadcaster(&nh);

  // Setup the chip select pin.
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  servo.attach(SERVO_PIN);

  // Allow time for initialization.
  delay(100);

  encoder->set_zero_point();

  delay(20);
}

void loop()
{
  // Transform and braodcast the value read from encoder.
  float encoded_angle = encoder->read_encoder();
  float tilt = encoded_angle * 2.0 * M_PI / 4096;

//  broadcaster->broadcast_transform_from_tilt(tilt);

  nh.spinOnce();

  int motor_speed;

  int idle = 90;
  if(scan_flag)
  {
    // and titled up corresponds to angles comming down from 360 while down goes up from 0
    // Check limiting conditions and reverse direction of speed if met.
    if(scan_dir_up && tilt < 330 && tilt > 180)
    {
      scan_dir_up = false;
      servo_speed = -10;
    }
    else if (!scan_dir_up && tilt > 30 && tilt < 180)
    {
      scan_dir_up = true;
      servo_speed = 10;
    }
    
    motor_speed = idle + servo_speed;
  }
  else
  {
    motor_speed = idle;
  }

  servo.write(motor_speed);
  
  test.data = "End of Loop.";
  test_pub.publish(&test);

  delay(10);
}

void enable_scan (const std_msgs::Bool& scan){
  scan_flag = scan.data;
}
