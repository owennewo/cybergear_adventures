/**
 * Sends random data and counter as tx extended messages.  Flashes LED if it receives a message.
 */

#include <Arduino.h>
#include "SimpleCAN.h"

uint32_t randomData = 0; // <- 32-bit unsigned is easy to use as can data (4 bytes)

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

  pinMode(LED_BUILTIN, OUTPUT);
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

  data = random_data();

  count++;
  if (count > 10)
  {
    count = 0;
    Serial.println("loop");
  }

  data[4] = count;

  delay(50);
  CanMsg txMsg = CanMsg(
      CanExtendedId(0x7FFE, false),
      5,
      data);
  delay(50);

  CAN.write(txMsg);
  delay(10);

  while (CAN.available() > 0)
  {
    CanMsg const rxMsg = CAN.read();

    if (rxMsg.isExtendedId())
    {
      blinkMany(12);
    }
    else
    {
      blinkMany(4);
    }
  }

  state = 1;
  blinkMany(2);
  delay(800);
  state = 2;
}
