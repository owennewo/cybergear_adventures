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
SPIClass spi2(P_SPI2_MOSI, P_SPI2_MISO, P_SPI2_SCK);            //, P_SPI2_CS);
MagneticSensorSPI sensor(AS5147_SPI, A_SPI2_CS);

Infineon6EDL7141Driver3PWM driver = Infineon6EDL7141Driver3PWM(A_INHA, A_INHB, A_INHC, A_SPI1_CS, false, A_EN_DRV);
BLDCMotor motor = BLDCMotor(14);

uint32_t pwmPeriod = 0;
uint32_t start_time = 0;

uint32_t watchA = 0;
uint32_t watchB = 0;
uint32_t watchC = 0;

float target_velocity = 0.1f;
float shaft_velocity = 0.0f;

static uint8_t data[8];

void sendMessage(uint32_t identifier, uint8_t *data, uint8_t length)
{
  CanMsg txMsg = CanMsg(
      CanExtendedId(identifier, false),
      length,
      data);

  CAN.write(txMsg);
}

void setup()
{
  initVariant();
  // pinMode(LED_BUILTIN, OUTPUT);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  CAN.begin(1000000);
  delay(1);

  sensor.init(&spi2);

  driver.pwm_frequency = 22000;
  driver.voltage_power_supply = 20; // cybergear can work from 10v-24V. 20V is a good value for testing
  driver.voltage_limit = 2.6;       // 2V is conservative for openloop, going much higher might cause excess heat
  driver.init(&spi1);
  DeadTimeConfiguration dt = driver.readDeadTimeConfiguration();

  dt.setDeadTimeFall(3240);
  dt.setDeadTimeRise(3240);

  uint16_t deadTimeRise = dt.getDeadTimeRise();
  uint16_t deadTimeFall = dt.getDeadTimeFall();

  data[0] = uint8_t(deadTimeRise >> 8);
  data[1] = uint8_t(deadTimeRise & 0xFF);
  data[2] = uint8_t(deadTimeFall >> 8);
  data[3] = uint8_t(deadTimeFall & 0xFF);
  data[4] = dt.DT_RISE;
  data[5] = dt.DT_FALL;

  sendMessage(0x100, data, 6);
  // dt.setDeadTimeFall(920);
  // dt.setDeadTimeRise(920);
  // driver.writeDeadTimeConfiguration(dt);

  motor.linkDriver(&driver);
  motor.voltage_limit = driver.voltage_limit / 2.0f;
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();

  driver.enable();

  start_time = millis();
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);

  pwmPeriod = ((GD32DriverParams *)driver.params)->period;

  motor.motion_downsample = 100;
}

uint8_t count = 0;
void loop()
{
  motor.move(target_velocity);
  // count++;
  // if (count == 0)
  // {
  sensor.update();
  shaft_velocity = -sensor.getVelocity();
  // }

  // watch vars + pwmPeriod be viewed in stm32cubemonitor in realtime
  watchA = driver.dc_a * pwmPeriod;
  watchB = driver.dc_b * pwmPeriod;
  watchC = driver.dc_c * pwmPeriod;
}
