/**
 * CYBERGEAR SPI1
 * --------------
 * SPI1 is connected to a single device, the Infineon 6EDL7141
 * This device has a 24bit word length, and is tricky to use with a gd32 which
 * is more comfortable with 8bit and 16bit words.
 *
 * This example sets SPI using gd32 libs
 *
 * The sample iterates through the first 10 registers trying to read 2 bytes.
 */
#include "SPI.h"
#include <Arduino.h>
#include "SimpleCAN.h"

#define SPI1_MOSI PORTB_15
#define SPI1_MISO PORTB_14
#define SPI1_SCK PORTB_13
#define SPI1_CS PORTB_12

SPIClass spi(SPI1_MOSI, SPI1_MISO, SPI1_SCK, SPI1_CS); //<- We can set CS here as no other SPI on this bus

SPISettings settings(1000000, MSBFIRST, SPI_MODE1);

void setup()
{

  CAN.begin(1000000);
  spi.begin();
}

static uint8_t data[8];

uint8_t reg = 0x01;

uint16_t receive0, receive1, receive2;
void loop()
{

  spi.beginTransaction(settings);
  receive0 = spi.transfer(reg);
  receive1 = spi.transfer(0x0);
  receive2 = spi.transfer(0x0);
  spi.endTransaction();

  data[0] = reg;
  data[1] = receive1;
  data[2] = receive2;

  bool isRtr = false;

  CanMsg txMsg = CanMsg(
      CanExtendedId(0b111111111111110, isRtr),
      3,
      data);

  CAN.write(txMsg);

  reg += 1;
  if (reg > 0x0A)
  {
    reg = 0x0;
  }

  delay(900);
}
