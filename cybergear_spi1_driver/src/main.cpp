/**
 * CYBERGEAR SPI1 DRIVER
 * ----------------------
 * SPI1 is connected to a single device, the Infineon 6EDL7141
 * This device has a 24bit word length with LOTS of status and configuration registers!!
 *
 * This prints out all the status registers.  It has a commented out example of reading/setting a config register.
 *
 */
#include "Wire.h"
#include <Arduino.h>
#include "SimpleCAN.h"
#include "SimpleFOC.h"
#include "drivers/infineon_6EDL7141/infineon_6EDL7141.h"
#include "PinNamesVar.h"

SPIClass spi1(P_SPI1_MOSI, P_SPI1_MISO, P_SPI1_SCK, P_SPI1_CS); //<- We can set CS here as no other SPI on this bus

SPISettings settings1(1000000, MSBFIRST, SPI_MODE0);

Infineon6EDL7141Driver3PWM driver = Infineon6EDL7141Driver3PWM(A_INHA, A_INHB, A_INHC, PB12, false, A_EN_DRV);

void setup()
{
  pinMode(A_EN_DRV, OUTPUT);
  digitalWrite(A_EN_DRV, LOW);
  pinMode(A_NFAULT, INPUT);
  CAN.begin(1000000);
  driver.init(&spi1);
  pinMode(P_SPI2_CS, OUTPUT);
  digitalWrite(P_SPI2_CS, HIGH);
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
  data[6] = status.PVDD_VAL;
  data[7] = uint8_t(status.getPVDDVoltage());

  sendMessage(0x02, data, 8);
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

void sendFAULTPin()
{
  data[0] = digitalRead(A_NFAULT); // <- 0 is a fault, 1 is no fault
  sendMessage(0xFF, data, 1);
}

void sendPowerSupplyConfig()
{
  PowerSupplyConfiguration configuration = driver.readPowerSupplyConfiguration();

  data[0] = configuration.PVCC_SETPT;
  data[1] = configuration.CS_REF_CFG;
  data[2] = configuration.DVDD_OCP_CFG;
  data[3] = configuration.DVDD_SFTSTRT;
  data[4] = configuration.DVDD_SETPT;
  data[5] = configuration.BK_FREQ;
  data[6] = configuration.DVDD_TON_DELAY;
  data[7] = configuration.CP_PRECHARGE_EN;

  sendMessage(0x11, data, 8);

  // delay(500);
  // configuration.setGateDrivingVoltage(GateDrivingVoltage::_7V);
  // configuration.setDVDDSoftStart(1600);
  // driver.writePowerSupplyConfiguration(configuration);

  // delay(500);
  // driver.clearFaults(true, true);
}

void loop()
{
  delay(900);
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
  sendPowerSupplyConfig();
  delay(900);
  sendDeadTimeConfiguration();
  delay(900);
  // updateDeadTimeConfiguration();
  // delay(900);
  // sendDeadTimeConfiguration();
  // delay(900);
  sendFAULTPin();
  delay(900);
}
