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
int pitch_up_lim = 330;
int pitch_down_lim = 45;

ros::NodeHandle nh;

std_msgs::Float32 lidar_angle;

ros::Publisher pub_encoder_angle("/lidar_mount/encoder_angle", &lidar_angle);
ros::Subscriber<std_msgs::Bool> sub_enable_scan("/lidar_mount/enable_scan",
    &enable_scan);

AbsoluteEncoder * encoder;

void setup()
{
  // Setup the ROS node.
  nh.initNode();
  nh.advertise(pub_encoder_angle);
  nh.subscribe(sub_enable_scan);

  // Test
  SPI.begin();

  encoder = new AbsoluteEncoder(CHIP_SELECT_PIN);

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
  // Publish the value read from encoder.
  float angle = encoder->read_encoder();
  lidar_angle.data = angle * 360 / 4096;
  pub_encoder_angle.publish(&lidar_angle);
  nh.spinOnce();

  int idle = 90;
  if(scan_flag)
  {
    // up corresponds to + speeds.
    // and titled up corresponds to angles comming down from 360 while down goes up from 0
  }
  else
  {
    servo.write(idle);
  }

  delay(10);
}

void enable_scan (const std_msgs::Bool& scan){
  scan_flag = scan.data;
}
