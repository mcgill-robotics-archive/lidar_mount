/*
 * Firmware implementation for Lidar Mount.
 *
 * Author: Malcolm Watt
 */
#include "firmware.h"

Servo servo;

// This boolean is used to signal whether or not we are reading.
// If we want to use the SPI bus to initialize, we need to set this to false.
bool g_reading = true;

int servo_position = 90;

ros::NodeHandle nh;

std_msgs::Float32 lidar_angle;
ros::Publisher pub_encoder_angle("/lidar/encoder_angle", &lidar_angle);
ros::Subscriber<std_msgs::Int32> sub_servo_position("/lidar/set_servo",
    &set_servo_position);

void setup()
{
  // Setup the ROS node.
  nh.initNode();
  nh.advertise(pub_encoder_angle);
  nh.subscribe(sub_servo_position);

  // Start the SPI library.
  SPI.begin();

  // Set up the servo.
  servo.attach(SERVO_PIN);

  // Setup the chip select pin.
  pinMode(CHIP_SELECT_PIN, OUTPUT);

  // Allow time for initialization.
  delay(100);

  set_zero_point();

  delay(20);
}

void loop()
{
  if (g_reading)
  {
    // Publish the value read from encoder.
    float angle = read_encoder();
    lidar_angle.data = angle;
    pub_encoder_angle.publish(&lidar_angle);
    nh.spinOnce();
  }

  servo.write(servo_position);

  nh.spinOnce();
  delay(1);
}

void set_servo_position (const std_msgs::Int32& angle){
  servo_position = angle.data;
  delay(1);
}

void initialize_transaction()
{
  digitalWrite(CHIP_SELECT_PIN, LOW);
}

void end_transaction()
{
  digitalWrite(CHIP_SELECT_PIN, HIGH);
}

void release_ss()
{
  delayMicroseconds(20);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  digitalWrite(CHIP_SELECT_PIN, LOW);
}

void nop_a5()
{
  initialize_transaction();
  SPI.transfer(NOP_A5);
  release_ss();
  end_transaction();
}

void set_zero_point()
{
  initialize_transaction();

  // Transfer the set_zero_point command.
  byte ack = SPI.transfer(SET_ZERO);
  // Transfer no_operation command untill we receive 0x80 which
  // is an acknowledgement that the encoder has been zeroed.
  while (ack != 0x80)
  {
    release_ss();
    ack = SPI.transfer(NOP_A5);
  }

  release_ss();

  end_transaction();
}

float read_encoder()
{
  initialize_transaction();

  // To hold transfer values.
  byte read_value;

  // Transfer the original read position command.
  read_value = SPI.transfer(RD_POS);

  /*
   * Transfer until we receive the RD_POS byte back. Based on the
   * data sheet, when we receive the RD_POS back, the next two bytes
   * contain the angle of the encoder.
   */
  while (read_value != RD_POS)
  {
    release_ss();
    read_value = SPI.transfer(NOP_A5);
  }

  /*
   * Based on the datasheet:
   * 1 -  send nop_a5 and receive MSB position (lower 4 bits of this byte are
   *      the upper 4 of the 12-bit position)
   * 2 -  Send second nop_a5 command and receive LSB position (lower 8 bits of
   *      12-bit positon)
   */
  release_ss();
  read_value = SPI.transfer(NOP_A5);

  unsigned int bit_angle = 0;

  bit_angle = (unsigned int)(read_value & 0x0F) * 256;

  release_ss();
  read_value = SPI.transfer(NOP_A5);

  bit_angle = bit_angle + (unsigned int) read_value;

  // Convert to degrees.
  float angle = (float) bit_angle;
  release_ss();
  end_transaction();

  return angle;
}
