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

std_msgs::Float32 lidar_angle;
std_msgs::Int32 test;
ros::Publisher pub_encoder_angle("/lidar_mount/encoder_angle", &lidar_angle);
ros::Subscriber<std_msgs::Bool> sub_enable_scan("/lidar_mount/enable_scan",
    &enable_scan);

ros::Publisher test_pub("/lidar_mount/test", &test);

AbsoluteEncoder * encoder;

void setup()
{
  // Setup the ROS node.
  nh.initNode();
  nh.advertise(pub_encoder_angle);
  nh.subscribe(sub_enable_scan);
  nh.advertise(test_pub);

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
  float angle_deg = angle * 360 / 4096;
  lidar_angle.data = angle;
  pub_encoder_angle.publish(&lidar_angle);
  nh.spinOnce();
    
  int motor_speed;

  int idle = 90;
  if(scan_flag)
  {
    // and titled up corresponds to angles comming down from 360 while down goes up from 0
    // Check limiting conditions and reverse direction of speed if met.
    if(scan_dir_up && angle_deg < 330 && angle_deg > 180)
    {
      scan_dir_up = false;
      servo_speed = -10;
    }
    else if (!scan_dir_up && angle_deg > 30 && angle_deg < 180)
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
  
  test.data = motor_speed;
  test_pub.publish(&test);

  delay(10);
}

void enable_scan (const std_msgs::Bool& scan){
  scan_flag = scan.data;
}
