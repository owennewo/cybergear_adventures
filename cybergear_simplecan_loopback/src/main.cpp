/**
 * LOOPBACK example, sends some data over loopback interface.  So will bounce stright back to can rx.
 * It isn't 'silent' so you'll also see it on canbus
 */

#include <Arduino.h>
#include "SimpleCAN.h"

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
  Serial.begin(115200);
  delay(1000);
  Serial.print("SysClock Speed: ");
  Serial.print(SystemCoreClock);
  Serial.println(" Hz");
  delay(10);

  pinMode(LED_BUILTIN, OUTPUT);
  CAN.enableInternalLoopback();
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

void loop()
{

  count++;
  if (count > 10)
  {
    count = 0;
    Serial.println("loop");
  }

  data = random_data();

  uint32_t txIdentifier = 0x3ff;
  bool isRtr = false;
  delay(50);
  CanMsg txMsg = CanMsg(
      isExtendedFrame ? CanExtendedId(txIdentifier, isRtr) : CanStandardId(txIdentifier, isRtr),
      4,
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

  blinkMany(2);
  delay(800);
}
