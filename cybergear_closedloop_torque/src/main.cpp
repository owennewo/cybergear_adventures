/**
 * CYBERGEAR VELOCITY CLOSED LOOP
 * ------------------------------
 * TO GET CLOSE LOOP Working we need to:
 * 1) have a working gd32 simplefoc hardware working (configuring tim0)
 * 2) Switch infineon to 3PWM mode (SPI1)
 * 3) AS5047 magnetic sensor needs to be linked to driver (SPI2)
 */
#include "Wire.h"
#include <Arduino.h>
#include "SimpleFOC.h"
#include "drivers/infineon_6EDL7141/infineon_6EDL7141.h"
#include "PinNamesVar.h"
#include "drivers/hardware_specific/gd32/gd32_mcu.h"
#include "SimpleCAN.h"
#include "can/CANCommander.h"
#include "can/modules/MetricsModule.h"
#include "can/modules/BaseModule.h"

SPIClass spi1(P_SPI1_MOSI, P_SPI1_MISO, P_SPI1_SCK, P_SPI1_CS); //<- We can set CS here as no other SPI on this bus
SPIClass spi2(P_SPI2_MOSI, P_SPI2_MISO, P_SPI2_SCK);            //, P_SPI2_CS);
MagneticSensorSPI sensor(AS5147_SPI, A_SPI2_CS);

Infineon6EDL7141Driver3PWM driver = Infineon6EDL7141Driver3PWM(A_INHA, A_INHB, A_INHC, A_SPI1_CS, false, A_EN_DRV);
BLDCMotor motor = BLDCMotor(14);

CANCommander commander = CANCommander();

uint32_t pwmPeriod = 0;

uint32_t start_time = 0;
float target_voltage = -0.95f;

void setup()
{
  initVariant();
  CAN.begin(1000000);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH); // Note: LED is backwards.  Pull LOW for on.

  commander.linkMotor(0x00, &motor);
  commander.monitor(0, BaseModule::MODULES::MODULE_METRICS, MetricsModule::FIELDS::M_METRICS_VELOCITY_ANGLE, 200);

  sensor.init(&spi2);

  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 20; // cybergear can work from 10v-24V. 20V is a good value for testing
  driver.voltage_limit = 4.0;       // 2V is conservative for openloop, going much higher might cause excess heat
  driver.init(&spi1);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 5;
  motor.LPF_velocity.Tf = 0.01;
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  motor.voltage_sensor_align = 4.0;
  motor.voltage_limit = driver.voltage_limit / 2;
  motor.controller = MotionControlType::velocity;

  motor.init();
  // motor.initFOC();
  start_time = millis();
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  motor.move(0);
  // pwmPeriod = ((GD32DriverParams *)driver.params)->period;
}

uint32_t watchA = 0;
uint32_t watchB = 0;
uint32_t watchC = 0;

float shaft_velocity = 0.0f;

void loop()
{
  commander.run();
  motor.loopFOC();
  motor.move();
}
