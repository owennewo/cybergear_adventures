#include <Arduino.h>
#include "SimpleCAN.h"
#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "drivers/infineon_6EDL7141/infineon_6EDL7141.h"
#include "PinNamesVar.h"

MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, PA15);
Infineon6EDL7141Driver3PWM driver = Infineon6EDL7141Driver3PWM(A_INHA,A_INHB,A_INHC, PB12, false, A_EN_DRV); 

uint32_t randomData = 0; // <- 32-bit unsigned is easy to use as can data (4 bytes)

uint32_t txState = 11;
extern uint16_t can_prescaler;
extern int can_bitrate;
extern uint8_t can_tseg1;
extern uint8_t can_tseg2;
extern uint8_t can_sjw;

// SPI_2 (mosi, miso, sclk)
SPIClass SPI2_SENSOR(PORTC_12, PORTC_11, PORTC_10);//, PORTA_15);
SPIClass SPI1_DRIVER(PORTB_15, PORTB_14, PORTB_13);

int acceptance_code = 0b111111111111110;
int acceptance_mask = 0b111111111111110;

bool isExtendedFrame = true;

void blinkMany(int times)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  delay(1000);
}

void outputHigh(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}


void setup()
{
  
outputHigh(PA1);
outputHigh(PB1);
outputHigh(PB2);
outputHigh(PB3);
// outputHigh(PB4);
outputHigh(PB5);
outputHigh(PB6);
outputHigh(PB7);
outputHigh(PB10);
outputHigh(PC6);
outputHigh(PC7);
outputHigh(PC8);
outputHigh(PC9);
outputHigh(PC14);
outputHigh(PC15);

  // RCU_APB1EN |= RCU_APB1EN_SPI2EN;
  rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4); 
// RCU_CFG0_ADCPSC(RCU_ADC_CKAPB2_DIV4)
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);
  rcu_periph_clock_enable(RCU_GPIOD);
  rcu_periph_clock_enable(RCU_ADC0);
  rcu_periph_clock_enable(RCU_ADC1);
  rcu_periph_clock_enable(RCU_ADC2);
  rcu_periph_clock_enable(RCU_TIMER0);
  rcu_periph_clock_enable(RCU_TIMER7);
  rcu_periph_clock_enable(RCU_TIMER1);
  rcu_periph_clock_enable(RCU_TIMER2);
  rcu_periph_clock_enable(RCU_TIMER3);
  rcu_periph_clock_enable(RCU_TIMER5);
  rcu_periph_clock_enable(RCU_AF);
  rcu_periph_clock_enable(RCU_SPI1);
  rcu_periph_clock_enable(RCU_SPI2);
  rcu_periph_clock_enable(RCU_CAN0);

  // gpio_pin_remap_config(GPIO_SPI2_REMAP, ENABLE);
  gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_12);
  gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
  Serial.begin(115200);
  delay(1000);
  // sensor.init(&SPI2_SENSOR);

  driver.init(&SPI1_DRIVER);

  delay(10);
  // gpio_pin_remap_config(GPIO_SPI2_REMAP, ENABLE);

  pinMode(LED_BUILTIN, OUTPUT);
  // CAN.enableInternalLoopback();
  // CanFilter filter = CanFilter(isExtendedFrame? MASK_EXTENDED: MASK_STANDARD, acceptance_code, acceptance_mask, FILTER_ANY_FRAME);
  // CAN.filter(filter);
  CAN.begin(1000000);
  gpio_pin_remap_config(GPIO_SPI2_REMAP, ENABLE);

  delay(10);
}

uint8_t data[4];

uint8_t *random_data()
{
  uint32_t randomNumber = random();

  static uint8_t data[4];
  data[0] = (randomNumber >> 24) & 0xFF; // Extract the most significant byte
  data[1] = (randomNumber >> 16) & 0xFF; // Extract the next byte
  data[2] = (randomNumber >> 8) & 0xFF;  // Extract the second least significant byte
  data[3] = randomNumber & 0xFF;
  return data;
}

uint8_t count = 0;
uint8_t state = 0;
float angle1 = 0.0f;
int16_t angle2 = 0;

void loop()
{

  count++;
  if (count > 10)
  {
    count = 0;
    Serial.println("loop");
  }

  int temp = driver.readTemperatureStatus().getTemperatureInCelsius();

  // sensor.update();
  // // display the angle and the angular velocity to the terminal
  // angle1 = sensor.getAngle();
  // // cast angle to int - to display in degrees
  // angle2 = (int16_t)(angle1 * 180 / M_PI);

data[0] = (uint8_t)(temp >> 8); // High byte
data[1] = (uint8_t)temp;

  // data[0] = temp;
  // data = random_data();

  uint32_t txIdentifier = acceptance_code;
  bool isRtr = false;
  delay(50);
  CanMsg txMsg = CanMsg(
      isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
      2,
      data);
  delay(50);


  CAN.write(txMsg);
  delay(10);

  while (CAN.available() > 0)
  {
    CanMsg const rxMsg = CAN.read();

    Serial.print("polling read: ");

    if (rxMsg.isExtendedId())
    {
      Serial.print(rxMsg.getExtendedId(), HEX);
      Serial.println(" Extended ✅");
      blinkMany(12);
    }
    else
    {
      Serial.print(rxMsg.getStandardId(), HEX);
      Serial.println(" Standard ✅");
      blinkMany(4);
    }
  }

  state = 1;
  blinkMany(2);
  delay(800);
  state = 2;
  
}
