/*
 * Reading of encoder and publishing to ROS.
 * 
 * TODO: Tidy up the SPI interface calls. Maybe use the provided lib...
 * 
 * Protocols are based on the datasheet for the CUI-AMT20 which
 * can be found at: http://www.cui.com/product/resource/amt20-v.pdf
 * 
 * Circuit:
 ** CSB : pin 10 (slave select)
 ** MOSI: pin 11 
 ** MISO: pin 12
 ** SCK : pin 13 (spi clock)
 * 
 * Author: Malcolm Watt
 */

#include<SPI.h>
#include <ros.h>
#include <std_msgs/Float32.h>

// Slave select pin.
const int CHIP_SEL_PIN = 10;

// SPI Parameter.
const long DATA_RATE = 1000000;

// Bytes for different command to interface with the encoder.
const byte NOP_A5 = 0x00;
const byte RD_POS = 0x10;
const byte SET_ZERO = 0x70;
const byte IDLE_CHAR = 0xA5;

// This boolean is used to signal whether or not we are reading.
// If we want to use the SPI bus to initialize, we need to set this to false.
bool g_reading = true;

ros::NodeHandle nh;

std_msgs::Float32 lidar_angle;
ros::Publisher lidar_mount("lidar_mount_angle", &lidar_angle);

void setup() 
{
  // Start the SPI library.
  SPI.begin();

  // Setup the chip select pin.
  pinMode(CHIP_SEL_PIN, OUTPUT);

  // Setup the ROS node.
  nh.initNode();
  nh.advertise(lidar_mount);

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
    lidar_mount.publish(&lidar_angle);
    nh.spinOnce();
  }
  delay(1);
}

/*
 * Initialize the SPI transaction.
 */
void initialize_transaction()
{
  SPI.beginTransaction (SPISettings (DATA_RATE, MSBFIRST, SPI_MODE0));
  digitalWrite(CHIP_SEL_PIN, LOW);
}

/*
 * Finalize the SPI transaction.
 */
void end_transaction()
{
  digitalWrite(CHIP_SEL_PIN, HIGH);
  SPI.endTransaction ();
}

/*
 * Thes particular encoder requires that we release the slave select line
 * every time we transmit a byte.
 * 
 * Also from the spec: "It is recommended that the master leave a 20
 * microsecond delay between reads to avoid extending the read time
 * by forcing wait sequences.
 */
void release_ss()
{
  delayMicroseconds(20);
  digitalWrite(CHIP_SEL_PIN, HIGH);
  digitalWrite(CHIP_SEL_PIN, LOW);
}

/*
 * Transfer a `no operation` command.
 */
void nop_a5()
{
  initialize_transaction();
  SPI.transfer(NOP_A5);
  end_transaction();
}

/*
 * Transfer a `zero set` command based on the CUI protocol.
 */
void set_zero_point()
{
  unsigned int count = 0;
  initialize_transaction();

  // Transfer the set_zero_point command.
  byte ack = SPI.transfer(SET_ZERO);
  // Transfer no_operation command untill we receive 0x80 which
  // is an acknowledgement that the encoder has been zeroed.
  while (ack != 0x80)
  {
    release_ss();
    ack = SPI.transfer(NOP_A5);
    count++;
  }

  end_transaction();
}

/*
 * Returns the angle of the encoder.
 * The encoder gives a 12-bit absolute position via SPI (4096 positions).
 */
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
   * 1 -  send nop_a5 and receive MSB position (lower 4 bits of this byte are the upper 4 of 
   *      the 12-bit position)
   * 2 -  Send second nop_a5 command and receive LSB position (lower 8 bits of 12-bit positon)
   */
  release_ss();
  read_value = SPI.transfer(NOP_A5);

  unsigned int bit_angle = 0;
  
  bit_angle = (unsigned int)(read_value & 0x0F) * 256;

  release_ss();
  read_value = SPI.transfer(NOP_A5);

  bit_angle = bit_angle + (unsigned int) read_value;

  // Convert to degrees.
  float angle = (float) bit_angle * 360 / 4096;

  end_transaction();

  return angle;
}
