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

    initialized = true;
}

void SPIClass::end()
{
    initialized = false;
}

void SPIClass::beginTransaction(SPISettings settings)
{
    digitalWrite(_ssel, LOW);
}

void SPIClass::endTransaction(void)
{
    digitalWrite(_ssel, HIGH);
}

uint8_t SPIClass::transfer(uint8_t val8)
{
    byte rec = 0;

    for (int i = 0; i < 8; i++)
    {
        digitalWrite(_mosi, bitRead(val8, 7 - i)); // MSB first
        digitalWrite(_sclk, HIGH);
        bitWrite(rec, 7 - i, digitalRead(_miso));
        digitalWrite(_sclk, LOW);
    }

    return rec;
}

uint16_t SPIClass::transfer16(uint16_t val16)
{
    uint16_t out_halfword;
    uint8_t trans_data0, trans_data1, rec_data0, rec_data1;

    trans_data0 = uint8_t(val16 & 0x00FF);
    trans_data1 = uint8_t((val16 & 0xFF00) >> 8);

    if (spisettings.bitorder == LSBFIRST)
    {
        rec_data0 = transfer(trans_data0);
        rec_data1 = transfer(trans_data1);
        out_halfword = uint16_t(rec_data0 || rec_data1 << 8);
    }
    else
    {
        rec_data0 = transfer(trans_data1);
        rec_data1 = transfer(trans_data0);
        out_halfword = uint16_t(rec_data1 || rec_data0 << 8);
    }

    return out_halfword;
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
    spi_begin(&_spi, spisettings.speed, spisettings.datamode, spisettings.bitorder);
}

void SPIClass::setDataMode(uint8_t mode)
{
    spisettings.datamode = mode;
    spi_begin(&_spi, spisettings.speed, spisettings.datamode, spisettings.bitorder);
}

void SPIClass::setClockDivider(uint32_t divider)
{
    if (divider == 0)
    {
        spisettings.speed = SPI_SPEED_DEFAULT;
    }
    else
    {
        /* Get clk freq of the SPI instance and compute it */
        spisettings.speed = dev_spi_clock_source_frequency_get(&_spi) / divider;
    }

    spi_begin(&_spi, spisettings.speed, spisettings.datamode, spisettings.bitorder);
}

void SPIClass::config(SPISettings settings)
{
    spisettings.speed = settings.speed;
    spisettings.datamode = settings.datamode;
    spisettings.bitorder = settings.bitorder;
}
