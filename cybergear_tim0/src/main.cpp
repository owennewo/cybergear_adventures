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
#include "drivers/hardware_specific/gd32/gd32_mcu.h"

SPIClass spi1(P_SPI1_MOSI, P_SPI1_MISO, P_SPI1_SCK, P_SPI1_CS); //<- We can set CS here as no other SPI on this bus

SPISettings settings1(1000000, MSBFIRST, SPI_MODE0);

Infineon6EDL7141Driver3PWM driver = Infineon6EDL7141Driver3PWM(A_INHA, A_INHB, A_INHC, A_SPI1_CS, false, A_EN_DRV);
BLDCMotor motor = BLDCMotor(14);

uint32_t pwmPeriod = 0;

uint32_t start_time = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  CAN.begin(1000000);
  delay(1);

  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 20; // cybergear can work from 10v-24V. 20V is a good value for testing
  driver.voltage_limit = 2.0;       // 2V is conservative for openloop, going much higher might cause excess heat
  driver.init(&spi1);

  motor.linkDriver(&driver);
  motor.voltage_limit = driver.voltage_limit / 2;
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();

  driver.enable();

  start_time = millis();
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);

  pwmPeriod = ((GD32DriverParams *)driver.params)->period;
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

uint32_t watchA = 0;
uint32_t watchB = 0;
uint32_t watchC = 0;

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

uint16_t angleA = 0;
uint16_t angleB = 120;
uint16_t angleC = 240;

uint8_t phaseA = 0;
uint8_t phaseB = 0;
uint8_t phaseC = 0;

float target_velocity = 1.0f;

void loop()
{

  // watch vars + pwmPeriod be viewed in stm32cubemonitor in realtime
  watchA = driver.dc_a * pwmPeriod;
  watchB = driver.dc_b * pwmPeriod;
  watchC = driver.dc_c * pwmPeriod;

  motor.move(target_velocity);
}
