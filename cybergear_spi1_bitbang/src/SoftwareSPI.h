#ifndef SoftwareSPI_h
#define SoftwareSPI_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class SoftwareSPI
{
public:
    SoftwareSPI(byte, byte, byte, byte);
    void begin();
    byte transfer(byte);
    void select();
    void deselect();

private:
    byte _pin_sck;
    byte _pin_mosi;
    byte _pin_miso;
    byte _pin_cs;
};

#endif