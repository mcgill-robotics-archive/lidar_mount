/*
 * Reading of encoder and publishing to ROS.
 *
 * Protocols are based on the datasheet for the CUI-AMT20 which
 * can be found at: http://www.cui.com/product/resource/amt20-v.pdf
 *
 * Circuit:
 *   --------------------------------------------------------
 * | Signal | Arduino pin | Encoder pin | Description of Sig  |
 * | ------ | ----------- | ----------- | ------------------- |
 * | CSB    | pin 10      | pin 2       | Slave Select        |
 * | MISO   | pin 12      | pin 3       | Master in Slave out |
 * | GND    | GND pin     | pin 4       |                     |
 * | SCK    | pin 13      | pin 5       | SPI Clock           |
 * | +5V    | 5 V pin     | pin 6       |                     |
 * | MOSI   | pin 11      | pin 7       | Master out Slave in |
 *   --------------------------------------------------------
 *
 * Author: Malcolm Watt
 */

#ifndef LIDAR_MOUNT_FIRMWARE
#define LIDAR_MOUNT_FIRMWARE

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>

// Arduino pins.
const int CHIP_SELECT_PIN = 10;


/*
 * Servo position intermediate method.
 */
void set_servo_position (const std_msgs::Int32& angle);

/*
 * Initialize the SPI transaction.
 */
void initialize_transaction();

/*
 * Finalize the SPI transaction.
 */
void end_transaction();

/*
 * Thes particular encoder requires that we release the slave select line
 * every time we transmit a byte.
 *
 * Also from the spec: "It is recommended that the master leave a 20
 * microsecond delay between reads to avoid extending the read time
 * by forcing wait sequences.
 */
void release_ss();

/*
 * Transfer a `no operation` command.
 */
void nop_a5();

/*
 * Transfer a `zero set` command based on the CUI protocol.
 */
void set_zero_point();

/*
 * Returns the angle of the encoder.
 * The encoder gives a 12-bit absolute position via SPI (4096 positions).
 */
float read_encoder();

#endif // LIDAR_MOUNT_FIRMWARE
