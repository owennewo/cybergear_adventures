#include <Arduino.h>
#include <SimpleFOC.h>
#include "PinNamesVar.h"

#define A_INHA PA8
#define A_INHB PA9
#define A_INHC PA10

BLDCDriver3PWM driver(A_INHA, A_INHB, A_INHC);

void setup()
{
  // pinMode(A_INHA, OUTPUT);
  // pinMode(A_INHB, OUTPUT);
  // pinMode(A_INHC, OUTPUT);
  // analogWriteResolution(256);
  // delay(10);
  driver.voltage_power_supply = 12;
  driver.init();
  driver.enable();
}

float valRed = 0.0f;
float valYellow = 0.3f;
float valBlue = 0.6f;
float valMax = 1.0f;

void loop()
{
  valRed += 0.1f;
  valYellow += 0.1f;
  valBlue += 0.1f;
  if (valRed > valMax)
    valRed = 0.0;
  if (valYellow > valMax)
    valYellow = 0.0;
  if (valBlue > valMax)
    valBlue = 0.0;
  driver.setPwm(valRed, valYellow, valBlue);
  delay(100);
}
