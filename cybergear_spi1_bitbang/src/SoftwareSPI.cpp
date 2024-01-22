#include "SoftwareSPI.h"

SoftwareSPI::SoftwareSPI(byte pin_sck, byte pin_mosi, byte pin_miso, byte pin_cs)
{
    _pin_sck = pin_sck;
    _pin_mosi = pin_mosi;
    _pin_miso = pin_miso;
    _pin_cs = pin_cs;
}

void SoftwareSPI::begin()
{
    deselect();
    pinMode(_pin_cs, OUTPUT);

    pinMode(_pin_sck, OUTPUT);
    pinMode(_pin_mosi, OUTPUT);
    pinMode(_pin_miso, INPUT);
}

byte SoftwareSPI::transfer(byte b)
{
    byte rec = 0;

    for (int i = 0; i < 8; i++)
    {
        digitalWrite(_pin_mosi, bitRead(b, 7 - i)); // MSB first
        digitalWrite(_pin_sck, HIGH);
        bitWrite(rec, 7 - i, digitalRead(_pin_miso));
        digitalWrite(_pin_sck, LOW);
    }

    return rec;
}

void SoftwareSPI::select()
{
    digitalWrite(_pin_cs, LOW);
}

void SoftwareSPI::deselect()
{
    digitalWrite(_pin_cs, HIGH);
}