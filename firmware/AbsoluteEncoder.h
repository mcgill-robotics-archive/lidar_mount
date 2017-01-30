/*
 * Library for an absolute encoder.
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
#ifndef ABS_ENC_FIRMWARE
#define ABS_ENC_FIRMWARE

#include <Arduino.h>
#include <SPI.h>


class AbsoluteEncoder
{
public:
  AbsoluteEncoder(const int CSB);

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

private:
  // SPI commands.
  static const byte NOP_A5 = 0x00;
  static const byte RD_POS = 0x10;
  static const byte SET_ZERO = 0x70;
  static const byte IDLE_CHAR = 0xA5;

  int SLAVE_SELECT;


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
};

#endif // ABS_ENC_FIRMWARE
