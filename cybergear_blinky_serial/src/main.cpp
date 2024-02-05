/**
 * Blinks the LED PIN, might send serial if you connect pin 16 and 17 (PA2 and PA3) - too hard for me to solder!
 *
 * Not LED is 'back to front' - so HIGH is off and LOW is on.
 */

#include <Arduino.h>

#define LED_PIN PA5

void setup()
{

  Serial.begin(115200);
  delay(3000);
  Serial.print("SysClock Speed: ");
  Serial.print(SystemCoreClock);
  Serial.println(" Hz");
  delay(10);

  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  digitalWrite(LED_PIN, LOW);
  delay(200);
  Serial.print("+");
  digitalWrite(LED_PIN, HIGH);
  delay(800);
  Serial.println("-");
}
