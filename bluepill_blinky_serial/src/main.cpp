#include <Arduino.h>

#define LED_PIN PB2

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
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  Serial.print("+");
  digitalWrite(LED_PIN, LOW);
  delay(2000);
  Serial.println("-");
}
