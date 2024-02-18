/**
 * This sample demonstrates the SPI2 peripheral which has a single AS50479 magnetic sensor connected to it.
 * It measures the angle (motor shaft position) and later will be used for closed loop motor control.
 *
 * There is no usart so use a debugger or stm monitor to check the angle.  Or you could listen to the CAN bus to see the angle.
 */

#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#include "PinNamesVar.h"
#include "SimpleFOC.h"
#include "SimpleCAN.h"

SPIClass spi2(P_SPI2_MOSI, P_SPI2_MISO, P_SPI2_SCK); //, P_SPI2_CS);

MagneticSensorSPI sensor(AS5147_SPI, A_SPI2_CS);

void setHigh(uint32_t gpio_periph, uint32_t pin)
{
  gpio_init(gpio_periph, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, pin); // LED PA5
  gpio_bit_set(GPIOA, pin);
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

void setup()
{
  // pinMode(PA15, OUTPUT);

  rcu_periph_clock_enable(RCU_AF);
  gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE); // we need PA15 to be released from JTAG otherwise it is pulled high
  CAN.begin(1000000);
  sensor.init(&spi2);
}

void blinkManyTimes(uint8_t count)
{

  for (int i = 1; i <= count; i++)
  {
    gpio_bit_reset(GPIOA, GPIO_PIN_5);
    delay(200);
    gpio_bit_set(GPIOA, GPIO_PIN_5);
    delay(200);
  }
}

float angle2 = 0.0f;

uint8_t tick = 0;
void loop()
{

  delay(1000);
  data[0] = tick;
  tick++;

  sensor.update();

  angle2 = sensor.getAngle();
  uint8_t angle_int = (uint8_t)(255.0f * angle2 / _PI_2);
  data[1] = angle_int;
  // data[2] = uint8_t(angle_int & 0xff);

  sendMessage(0x11, data, 3);
}
