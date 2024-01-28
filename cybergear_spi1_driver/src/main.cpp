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
#include "Wire.h"
#include <Arduino.h>
#include "SimpleCAN.h"
#include "drivers/infineon_6EDL7141/infineon_6EDL7141.h"
#include "PinNamesVar.h"

PinName SPI1_MOSI = PORTB_15;
PinName SPI1_MISO = PORTB_14;
PinName SPI1_SCK = PORTB_13;
PinName SPI1_CS = PORTB_12;

PinName SPI2_MOSI = PORTC_12;
PinName SPI2_MISO = PORTC_11;
PinName SPI2_SCK = PORTC_10;
PinName SPI2_CS = PORTA_15;

// SPIClass spi(DIGITAL_TO_PINNAME(A_SPI1_MOSI), DIGITAL_TO_PINNAME(A_SPI1_MISO), DIGITAL_TO_PINNAME(A_SPI1_SCK), DIGITAL_TO_PINNAME(A_SPI1_CS));
SPIClass spi(SPI1_MOSI, SPI1_MISO, SPI1_SCK, SPI1_CS);
// SPIClass spi(SPI2_MOSI, SPI2_MISO, SPI2_SCK, SPI2_CS);

// Infineon6EDL7141Driver3PWM driver = Infineon6EDL7141Driver3PWM(A_INHA, A_INHB, A_INHC, PB12, false, A_EN_DRV);

void setup()
{
  pinMode(PB15, OUTPUT);
  pinMode(PB14, INPUT);
  pinMode(PB13, OUTPUT);
  pinMode(PB12, OUTPUT);

  pinMode(PC12, OUTPUT);
  pinMode(PC11, INPUT);
  pinMode(PC10, OUTPUT);
  pinMode(PA15, OUTPUT);

  CAN.begin(1000000);
  spi.begin();
  // driver.init(&spi);
}

static uint8_t data[8];

// uint8_t temp_reg = 0x3FFF; // TEMP STATUS
uint8_t temp_reg = 0x1; // TEMP STATUS

void loop()
{

  spi.transfer(temp_reg);

  data[0] = uint8_t(temp_reg);
  spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  spi.transfer(temp_reg);
  // spi.transfer(0x3f);
  // spi.transfer(0xff);
  data[1] = spi.transfer(0x0);
  data[2] = spi.transfer(0x0);
  spi.endTransaction();
  // data[3] = uint8_t(reg2 & 0xFF);
  // data[4] = uint8_t(reg3 >> 8);
  // data[5] = uint8_t(reg3 & 0xFF);
  // data[6] = uint8_t(reg4 >> 8);
  // data[7] = uint8_t(reg4 & 0xFF);

  // temp_reg += 1;
  // if (temp_reg > 0x09)
  // {
  //   temp_reg = 0x00;
  // }

  bool isRtr = false;
  CanMsg txMsg = CanMsg(
      CanExtendedId(0b111111111111110, isRtr),
      3,
      data);

  CAN.write(txMsg);

  delay(900);

  // while (true)
  //   ;
}
