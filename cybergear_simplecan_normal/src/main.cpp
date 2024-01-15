#include <Arduino.h>
#include "SimpleCAN.h"


uint32_t randomData = 0; // <- 32-bit unsigned is easy to use as can data (4 bytes)

uint32_t txState = 11;
extern uint16_t can_prescaler;
extern int can_bitrate;
extern uint8_t can_tseg1;
extern uint8_t can_tseg2;
extern uint8_t can_sjw;


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


void setup()
{
  pinMode(PB4, OUTPUT);
  digitalWrite(PB4, HIGH); // this might enable CAN
  Serial.begin(115200);
  delay(1000);
  Serial.print("SysClock Speed: ");
  Serial.print(SystemCoreClock);
  Serial.println(" Hz");
  delay(10);

  pinMode(LED_BUILTIN, OUTPUT);
  // CAN.enableInternalLoopback();
  // CanFilter filter = CanFilter(isExtendedFrame? MASK_EXTENDED: MASK_STANDARD, acceptance_code, acceptance_mask, FILTER_ANY_FRAME);
  // CAN.filter(filter);
  CAN.begin(1000000);
  delay(10);
}

uint8_t *data = nullptr;

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

void loop()
{

  count++;
  if (count > 10)
  {
    count = 0;
    Serial.println("loop");
  }

  data = random_data();

  uint32_t txIdentifier = acceptance_code;
  bool isRtr = false;
  delay(50);
  CanMsg txMsg = CanMsg(
      isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
      0,
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
