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

Infineon6EDL7141Driver3PWM driver = Infineon6EDL7141Driver3PWM(A_INHA, A_INHB, A_INHC, A_SPI1_CS, false, A_EN_DRV);
BLDCMotor motor = BLDCMotor(14);

float maxVoltage = 127.0f;

uint32_t start_time = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(A_EN_DRV, OUTPUT);
  digitalWrite(A_EN_DRV, LOW);
  pinMode(A_NFAULT, INPUT);
  CAN.begin(1000000);
  // driver.init(&spi1);
  pinMode(P_SPI2_CS, OUTPUT);
  digitalWrite(P_SPI2_CS, HIGH);
  rcu_periph_clock_enable(RCU_TIMER0);
  delay(1);

  /**
   * TIMER0 is connected to apb2 bus.  It looks like RCU_CFG0/APB2PSC = 0b000 which means timer clock = AHB i.e. 120Mhz
   * With auto reload register set to 5000, and prescaler set to 0, the PWM frequency is 120Mhz / 5000 = 24Khz
   */

  // // TIMER_CTL0(TIMER0) = 0x00000011;
  // TIMER_CTL0(TIMER0) = 0x000000D1; // <- dir not taking, but probably not important as dir isn't important in center mode
  // // bit 0 = 1 (enable timer)
  // // bit 4 = 1 (DIR count down)
  // // bit 5,6 = 10 (center aligned, counting up)

  // TIMER_CTL1(TIMER0) = 0x00000020;
  // TIMER_SMCFG(TIMER0) = 0x00000080; // Master slave mode enabled (other times may synchronise to this one)

  // // TIMER_DMAINTEN(TIMER0) = 0x00000001;

  // // // TIMER_INTF(TIMER0) = 0x0000001E; // <- hardware sets these, software can clear them
  // // // TIMER_SWEVG(TIMER0) = 0x0; // write only?? capture compare event generation

  // TIMER_CHCTL0(TIMER0) = 0x00006868; // setting ch0 and ch1 to PWM mode
  // TIMER_CHCTL1(TIMER0) = 0x00000068; // setting ch2 to PWM mode
  // TIMER_CHCTL2(TIMER0) = 0x00000BBB; // ch0, ch1, ch2 enabled, active low,

  // // // TIMER_CNT(TIMER0) = 0x000012E7; // the value of counter (don't need to set this)
  // // TIMER_PSC(TIMER0) = 0x0;    // timer cloc divider 0x0 = 1
  // TIMER_CAR(TIMER0) = 0x1388; // <- auto reload is 5000

  // // // ?? 0x30 is out of order
  // TIMER_CREP(TIMER0) = 0x00000001;
  // TIMER_CH0CV(TIMER0) = 0x000009C4;
  // TIMER_CH1CV(TIMER0) = 0x000009C4;
  // TIMER_CH2CV(TIMER0) = 0x000009C4;
  // TIMER_CH3CV(TIMER0) = 0x0;
  // // // 0x44 is out of orders
  // // TIMER_CCHP(TIMER0) = 0x00008000;
  // // TIMER_DMACFG(TIMER0) = 0x0;
  // // TIMER_DMATB(TIMER0) = 0x000000C1;
  // // // missing
  // // // TIMER_IRMP(TIMER0) = 0x0;
  // // // missing
  // // // TIMER_CFG(TIMER0) = 0x0;

  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 0.5;

  // driver init
  driver.init(&spi1);
  motor.linkDriver(&driver);
  motor.voltage_limit = driver.voltage_limit / 2;
  motor.velocity_limit = 0.5;

  motor.controller = MotionControlType::velocity_openloop;

  motor.init();

  driver.enable();

  start_time = millis();
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);

  driver.setPwm(0.0, 2.0, 2.0);
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

float target_velocity = 2.5f;

void loop()
{

  // sendPowerSupplyStatus();
  // delay(900);

  angleA += 1;
  angleB += 1;
  angleC += 1;
  angleA = angleA % 360;
  angleB = angleB % 360;
  angleC = angleC % 360;

  float angleRadA = angleA * 3.14159 / 180.0;
  float angleRadB = angleB * 3.14159 / 180.0;
  float angleRadC = angleC * 3.14159 / 180.0;

  float voltageA = 127 + maxVoltage * sin(angleRadA);
  float voltageB = 127 + maxVoltage * sin(angleRadB);
  float voltageC = 127 + maxVoltage * sin(angleRadC);

  // sendMessage(0x01, data, 3);

  phaseA = uint8_t(255.0f * driver.dc_a);
  phaseB = uint8_t(255.0f * driver.dc_b);
  phaseC = uint8_t(255.0f * driver.dc_c);

  // delay(900);
  // sendFaultStatus();
  // delay(900);
  // sendTemperatureStatus();
  // delay(900);
  // sendFunctionalStatus();
  // delay(900);
  // sendOTPStatus();
  // delay(900);
  // sendADCStatus();
  // delay(900);
  // sendChargePumpsStatus();
  // delay(900);
  // sendDeviceID();
  // delay(900);
  // sendPowerSupplyConfig();
  // delay(900);
  // sendDeadTimeConfiguration();
  // delay(900);
  // // updateDeadTimeConfiguration();
  // // delay(900);
  // // sendDeadTimeConfiguration();
  // // delay(900);
  // sendFAULTPin();
  // delay(900);

  // motor.move(target_velocity);

  if (millis() - start_time > 4000)
  {
    driver.setPwm(0.0, 0.0, 0.0);
    digitalWrite(LED_BUILTIN, LOW);
    driver.disable();

    while (true)
      ;
  }
  else if (millis() - start_time > 2000)
  {
    // driver.setPwm(1.0, 0.0, 0.0);
  }
}
