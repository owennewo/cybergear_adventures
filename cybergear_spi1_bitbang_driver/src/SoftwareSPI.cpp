#include "SoftwareSPI.h"
#include <Arduino.h>
#include <assert.h>
#include "pins_arduino.h"

SPIClass SPI;

SPIClass::SPIClass(void)
{
    _spi.pin_miso = DIGITAL_TO_PINNAME(MISO);
    _spi.pin_mosi = DIGITAL_TO_PINNAME(MOSI);
    _spi.pin_sclk = DIGITAL_TO_PINNAME(SCK);
    _spi.pin_ssel = NC;

    _miso = PinName_to_digital(_spi.pin_miso);
    _mosi = PinName_to_digital(_spi.pin_mosi);
    _sclk = PinName_to_digital(_spi.pin_sclk);
    _ssel = NC;

    _clockidle = LOW;
    _clockactive = HIGH;
    initialized = false;
}

SPIClass::SPIClass(PinName mosi, PinName miso, PinName sclk, PinName ssel)
{
    _spi.pin_miso = miso;
    _spi.pin_mosi = mosi;
    _spi.pin_sclk = sclk;
    _spi.pin_ssel = ssel;

    _miso = PinName_to_digital(miso);
    _mosi = PinName_to_digital(mosi);
    _sclk = PinName_to_digital(sclk);
    _ssel = PinName_to_digital(ssel);

    initialized = false;
}

SPIClass::SPIClass(PinName mosi, PinName miso, PinName sclk)
{
    _spi.pin_miso = miso;
    _spi.pin_mosi = mosi;
    _spi.pin_sclk = sclk;
    _spi.pin_ssel = NC;

    _miso = PinName_to_digital(miso);
    _mosi = PinName_to_digital(mosi);
    _sclk = PinName_to_digital(sclk);
    _ssel = NC;

    initialized = false;
}

void SPIClass::begin()
{
    uint32_t test = 0;
    if (initialized)
    {
        return;
    }
    pinMode(_sclk, OUTPUT);
    digitalWrite(_sclk, HIGH);
    pinMode(_ssel, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);

    _clockidle = (spisettings.datamode == SPI_MODE0 || spisettings.datamode == SPI_MODE1) ? LOW : HIGH;
    _clockactive = _clockidle == LOW ? HIGH : LOW;
    digitalWrite(_sclk, _clockidle);
    initialized = true;
}

void SPIClass::end()
{
    initialized = false;
}

void SPIClass::beginTransaction(SPISettings settings)
{
    digitalWrite(_ssel, LOW);
    delayMicroseconds(1);
}

void SPIClass::endTransaction(void)
{
    digitalWrite(_ssel, HIGH);
    delayMicroseconds(1);
}

uint8_t SPIClass::transfer(uint8_t val8)
{
    byte rec = 0;

    for (int i = 0; i < 8; i++)
    {
        uint8_t currentBit = spisettings.bitorder == MSBFIRST ? 7 - i : i;

        if (spisettings.datamode == SPI_MODE0 || spisettings.datamode == SPI_MODE2)
        {
            digitalWrite(_sclk, _clockactive); // First edge
            delayMicroseconds(1);
            bitWrite(rec, currentBit, digitalRead(_miso));
            digitalWrite(_sclk, _clockidle); // Second edge
            delayMicroseconds(1);
        }
        else // SPI_MODE1 or SPI_MODE3
        {
            digitalWrite(_sclk, _clockidle); // First edge
            delayMicroseconds(1);
            bitWrite(rec, currentBit, digitalRead(_miso));
            digitalWrite(_sclk, _clockactive); // Second edge
            delayMicroseconds(1);
        }
    }

    digitalWrite(_sclk, _clockidle); // Ensure clock ends up in idle state (for mode 1 and 3)

    return rec;
}

uint16_t SPIClass::transfer16(uint16_t val16)
{
    uint8_t trans_data0, trans_data1, rec_data0, rec_data1;

    trans_data0 = uint8_t(val16 & 0x00FF);
    trans_data1 = uint8_t(val16 >> 8);

    rec_data0 = transfer(trans_data0);
    rec_data1 = transfer(trans_data1);
    if (spisettings.bitorder == MSBFIRST)
    {
        return (uint16_t(rec_data0) << 8) | uint16_t(rec_data1);
    }
    else
    {
        return (uint16_t(rec_data1) << 8) | uint16_t(rec_data0);
    }
}

void SPIClass::transfer(void *buf, size_t count)
{
    for (int i = 0; i < count; i++)
    {
        transfer(((uint8_t *)buf)[i]);
    }
}

void SPIClass::transfer(void *bufout, void *bufin, size_t count)
{
    for (int i = 0; i < count; i++)
    {
        ((uint8_t *)bufin)[i] = transfer(((uint8_t *)bufout)[i]);
    }
}

void SPIClass::setBitOrder(BitOrder order)
{
    spisettings.bitorder = order;
}

void SPIClass::setDataMode(uint8_t mode)
{
    spisettings.datamode = mode;
}

void SPIClass::setClockDivider(uint32_t divider)
{
    // TODO: Implement speed.  It'll probably be running at ~400mbs due to 1ms delays
    spisettings.speed = F_CPU / divider;
}

void SPIClass::config(SPISettings settings)
{
    spisettings.speed = settings.speed;
    spisettings.datamode = settings.datamode;
    spisettings.bitorder = settings.bitorder;
}
