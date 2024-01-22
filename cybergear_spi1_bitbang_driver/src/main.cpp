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
#include "SoftwareSPI.h"
#include "Wire.h"
#include <Arduino.h>
#include "SimpleCAN.h"
#include "drivers/infineon_6EDL7141/infineon_6EDL7141.h"
#include "PinNamesVar.h"

PinName SPI1_MOSI = PORTB_15;
PinName SPI1_MISO = PORTB_14;
PinName SPI1_SCK = PORTB_13;
PinName SPI1_CS = PORTB_12;
// SPIClass spi(DIGITAL_TO_PINNAME(A_SPI1_MOSI), DIGITAL_TO_PINNAME(A_SPI1_MISO), DIGITAL_TO_PINNAME(A_SPI1_SCK), DIGITAL_TO_PINNAME(A_SPI1_CS));
SPIClass spi(SPI1_MOSI, SPI1_MISO, SPI1_SCK, SPI1_CS);

Infineon6EDL7141Driver3PWM driver = Infineon6EDL7141Driver3PWM(A_INHA, A_INHB, A_INHC, PB12, false, A_EN_DRV);

void setup()
{
  CAN.begin(1000000);
  spi.begin();
  // driver.init(&spi);
}

static uint8_t data[8];

uint8_t temp_reg = 0x01; // TEMP STATUS

void loop()
{
  spi.beginTransaction(DEFAULT_SPI_SETTINGS);
  uint8_t ignore1 = spi.transfer(temp_reg); // first 8 bytes target the register
  uint8_t ignore2 = spi.transfer(0x00);     // for temp status, nothing is in first byte
  uint8_t raw_temp = spi.transfer(0x00);    // first 7 bits of second byte is temp
  spi.endTransaction();

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
