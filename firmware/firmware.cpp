/*
 * Firmware implementation for Lidar Mount.
 *
 * Author: Malcolm Watt
 */

#include "firmware.h"

// Arduino pins.
const int SERVO_PIN = 6;
const int CHIP_SELECT_PIN = 10;

void enable_scan(const std_msgs::Bool& scan);

Servo servo;

ros::NodeHandle nh;

std_msgs::Float32 lidar_angle;
std_msgs::Int32 test;
ros::ServiceServer<lidar_mount::LidarScan::Request, lidar_mount::LidarScan::Response> scanSrv(
        "/lidar_mount/set_scan_flag",
        &ScanController::setScanModeCallback
);
ros::Publisher pub_encoder_angle("/lidar_mount/encoder_angle", &lidar_angle);
ros::Publisher test_pub("/lidar_mount/test", &test);

AbsoluteEncoder * encoder;

void setup()
{
  // Setup the ROS node.
  nh.initNode();
  nh.advertise(pub_encoder_angle);
  nh.advertise(test_pub);
  nh.advertiseService(scanSrv);

  // Test
  SPI.begin();

  encoder = new AbsoluteEncoder(CHIP_SELECT_PIN);

  // Setup the chip select pin.
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  servo.attach(SERVO_PIN);

  ScanController::scan_flag = false;
  ScanController::scan_dir_up = true;
  ScanController::idle = 90;
  ScanController::servo_speed = 10;
  ScanController::pitch_up_lim = 335;
  ScanController::pitch_down_lim = 25;

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
  lidar_angle.data = angle_deg;
  pub_encoder_angle.publish(&lidar_angle);
  nh.spinOnce();

  int motor_speed = ScanController::getMotorCommand(angle_deg);

  servo.write(motor_speed);

  test.data = motor_speed;
  test_pub.publish(&test);

  delay(10);
}
