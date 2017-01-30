#include "AbsoluteEncoder.h"


AbsoluteEncoder::AbsoluteEncoder(const int CSB)
{
  SLAVE_SELECT = CSB;
}


void AbsoluteEncoder::initialize_transaction()
{
  digitalWrite(SLAVE_SELECT, LOW);
}


void AbsoluteEncoder::end_transaction()
{
  digitalWrite(SLAVE_SELECT, HIGH);
}


void AbsoluteEncoder::release_ss()
{
  delayMicroseconds(20);
  digitalWrite(SLAVE_SELECT, HIGH);
  digitalWrite(SLAVE_SELECT, LOW);
}


void AbsoluteEncoder::nop_a5()
{
  initialize_transaction();
  SPI.transfer(NOP_A5);
  release_ss();
  end_transaction();
}


void AbsoluteEncoder::set_zero_point()
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


float AbsoluteEncoder::read_encoder()
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
