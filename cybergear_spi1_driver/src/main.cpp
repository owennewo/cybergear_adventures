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

#define SPI1_MOSI PORTB_15
#define SPI1_MISO PORTB_14
#define SPI1_SCK PORTB_13
#define SPI1_CS PORTB_12

SPIClass spi(SPI1_MOSI, SPI1_MISO, SPI1_SCK, SPI1_CS); //<- We can set CS here as no other SPI on this bus

SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

Infineon6EDL7141Driver3PWM driver = Infineon6EDL7141Driver3PWM(A_INHA, A_INHB, A_INHC, PB12, false, A_EN_DRV);

void setup()
{

  CAN.begin(1000000);
  driver.init(&spi);
}

static uint8_t data[8];

void sendMessage(uint32_t identifier, uint8_t *data, uint8_t length)
{
  CanMsg txMsg = CanMsg(
      CanExtendedId(identifier, false),
      length,
      data);

  CAN.write(txMsg);
}

void sendFaultStatus()
{
  FaultAndWarningStatus status = driver.readFaultAndWarningStatus();

  data[0] = 0;
  data[1] = status.isCurrentSenseOverCurrentPhaseA() ? 1 : 0;
  data[2] = status.isCurrentSenseOverCurrentPhaseB() ? 1 : 0;
  data[3] = status.isCurrentSenseOverCurrentPhaseC() ? 1 : 0;
  data[4] = status.isChargePump() ? 1 : 0;
  data[5] = status.isDVDDOverCurrent() ? 1 : 0;
  data[6] = status.isDVDDUnderVoltage() ? 1 : 0;

  sendMessage(0x00, data, 7);
  data[0] = 1;
  data[1] = status.isBuckOverCurrentProtection() ? 1 : 0;
  data[2] = status.isOverTemperatureShutdown() ? 1 : 0;
  data[3] = status.isOverTemperatureWarning() ? 1 : 0;
  data[4] = status.isLockedRotor() ? 1 : 0;
  data[5] = status.isWatchdog() ? 1 : 0;
  data[6] = status.isOTPMemory() ? 1 : 0;
  sendMessage(0x00, data, 7);
}

void sendTemperatureStatus()
{
  data[0] = driver.readTemperatureStatus().getTemperatureInCelsius();
  sendMessage(0x01, data, 1);
}

void sendPowerSupplyStatus()
{
  PowerSupplyStatus status = driver.readPowerSupplyStatus();

  data[0] = status.isChargePumpLowSideUnderVoltage() ? 1 : 0;
  data[1] = status.isChargePumpHighSideUnderVoltage() ? 1 : 0;
  data[2] = status.isDVDDUnderVoltage() ? 1 : 0;
  data[3] = status.isDVDDOverVoltage() ? 1 : 0;
  data[4] = status.isVDDBUnderVoltage() ? 1 : 0;
  data[5] = status.isVDDBOverVoltage() ? 1 : 0;
  data[6] = status.getPVDDVoltage();

  sendMessage(0x02, data, 7);
}

void sendFunctionalStatus()
{
  FunctionalStatus status = driver.readFunctionalStatus();

  data[0] = status.getHallStatePhaseA() ? 1 : 0;
  data[1] = status.getHallStatePhaseB() ? 1 : 0;
  data[2] = status.getHallStatePhaseC() ? 1 : 0;
  data[3] = status.isHallPolarityEqual() ? 1 : 0;
  data[4] = status.isDVDDSetPoint5V() ? 1 : 0;
  data[5] = status.getCurrentSenseGain();

  sendMessage(0x03, data, 6);
}

void sendOTPStatus()
{
  OTPStatus status = driver.readOTPStatus();

  data[0] = status.isOneTimeProgramUsed() ? 1 : 0;
  data[1] = status.isOneTimeProgramPassed() ? 1 : 0;
  data[2] = status.isOneTimeProgramBlocked() ? 1 : 0;
  data[3] = status.isOneTimeProgramFailed() ? 1 : 0;

  sendMessage(0x04, data, 4);
}

void sendADCStatus()
{
  ADCStatus status = driver.readADCStatus();

  data[0] = status.isReady() ? 1 : 0;
  data[1] = status.getValue();

  sendMessage(0x05, data, 2);
}

void sendChargePumpsStatus()
{
  ChargePumpsStatus status = driver.readChargePumpsStatus();

  data[0] = status.getChargePumpHighSideVoltage();
  data[1] = status.getChargePumpLowSideVoltage();

  sendMessage(0x06, data, 2);
}

void sendDeviceID()
{
  DeviceID status = driver.readDeviceID();

  data[0] = status.DEV_ID;

  sendMessage(0x07, data, 1);
}

void sendDeadTimeConfiguration()
{
  DeadTimeConfiguration configuration = driver.readDeadTimeConfiguration();

  data[0] = configuration.DT_RISE;
  data[1] = configuration.DT_FALL;

  sendMessage(0x1B, data, 2);
}

void updateDeadTimeConfiguration()
{
  DeadTimeConfiguration configuration = driver.readDeadTimeConfiguration();

  configuration.setDeadTimeRise(360);
  configuration.setDeadTimeFall(440);

  driver.writeDeadTimeConfiguration(configuration);
}

void loop()
{
  sendFaultStatus();
  delay(900);
  sendTemperatureStatus();
  delay(900);
  sendPowerSupplyStatus();
  delay(900);
  sendFunctionalStatus();
  delay(900);
  sendOTPStatus();
  delay(900);
  sendADCStatus();
  delay(900);
  sendChargePumpsStatus();
  delay(900);
  sendDeviceID();
  delay(900);

  sendDeadTimeConfiguration();
  delay(900);
  updateDeadTimeConfiguration();
  delay(900);
  sendDeadTimeConfiguration();
  delay(900);

  while (true)
    ;
}
