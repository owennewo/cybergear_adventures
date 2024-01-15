#include <Arduino.h>
#include "SimpleCAN.h"

#define LED_PIN PB2

uint32_t randomData = 0; // <- 32-bit unsigned is easy to use as can data (4 bytes)

int acceptance_code = 0x321;
int acceptance_mask = 0x321;
bool isExtendedFrame = false;


void setup()
{

  Serial.begin(115200);
  delay(4000);
  Serial.println("Starting CAN");
  delay(10);

  pinMode(LED_PIN, OUTPUT);

  CAN.logTo(&Serial);
  delay(2000);
  Serial.println("Starting CAN");
  CAN.enableInternalLoopback();
  CanFilter filter = CanFilter(isExtendedFrame? MASK_EXTENDED: MASK_STANDARD, acceptance_code, acceptance_mask, FILTER_ANY_FRAME);
  // CanFilter filter = CanFilter(MASK_EXTENDED, 0x321, 0x321, FILTER_ANY_FRAME);

  CAN.filter(filter);
  CAN.begin(125000);
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

void loop()
{
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  // test_flag_interrupt = can_loopback_transmit(useInterrupt);

  data = random_data();

  uint32_t txIdentifier = acceptance_code;
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
    }
    else
    {
      Serial.print(rxMsg.getStandardId(), HEX);
      Serial.println(" Standard ✅");
    }
  }

  digitalWrite(LED_PIN, LOW);
  delay(800);
}
