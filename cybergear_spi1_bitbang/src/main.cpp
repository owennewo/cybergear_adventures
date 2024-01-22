/**
 * CYBERGEAR SPI1 BITBANG
 * ----------------------
 * SPI1 is connected to a single device, the Infineon 6EDL7141
 * This device has a 24bit word length, and is tricky to use with a gd32 which
 * is more comfortable with 8bit and 16bit words.
 *
 * This example uses the SoftwareSPI library to bitbang the SPI1 interface which can handle
 * various word lengths.
 *
 */

#include <Arduino.h>
#include "SimpleCAN.h"
#include "SoftwareSPI.h"

SoftwareSPI spi(PB13, PB15, PB14, PB12);

void setup()
{
  CAN.begin(1000000);
  spi.begin();
}

static uint8_t data[8];

uint8_t temp_reg = 0x01; // TEMP STATUS

void loop()
{
  spi.select();
  uint8_t ignore1 = spi.transfer(temp_reg); // first 8 bytes target the register
  uint8_t ignore2 = spi.transfer(0x00);     // for temp status, nothing is in first byte
  uint8_t raw_temp = spi.transfer(0x00);    // first 7 bits of second byte is temp
  spi.deselect();

  data[0] = temp_reg;
  data[1] = raw_temp;
  data[2] = raw_temp * 2 - 94; // convert to celsius, as per datasheet

  bool isRtr = false;
  CanMsg txMsg = CanMsg(
      CanExtendedId(0b111111111111110, isRtr),
      3,
      data);

  CAN.write(txMsg);

  delay(900);
}
