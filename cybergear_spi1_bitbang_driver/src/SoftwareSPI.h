/*
 * SoftwareSPI.h
 * If this is included before Arduino.h then this will override the SPIClass
 *
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include "Arduino.h"
#include <stdio.h>
extern "C"
{
#include "utility/drv_spi.h"
}

/*
 * SPI_HAS_TRANSACTION means SPI has
 *  - beginTransaction()
 *  - endTransaction()
 *  - SPISetting(clock, bitorder, datamode)
 */
#define SPI_HAS_TRANSACTION 1

class SPISettings
{
public:
    SPISettings(uint32_t speedMax, BitOrder bitOrder, uint8_t dataMode)
    {
        this->speed = speedMax;
        this->bitorder = bitOrder;
        this->datamode = dataMode;
    }

    /* Set speed to default, SPI mode set to MODE 0 and Bit order set to MSB first. */
    SPISettings()
    {
        this->speed = SPI_SPEED_DEFAULT;
        this->bitorder = MSBFIRST;
        this->datamode = SPI_MODE0;
    }

private:
    uint32_t speed;
    uint8_t datamode;
    BitOrder bitorder;

    friend class SPIClass;
};

const SPISettings DEFAULT_SPI_SETTINGS = SPISettings();

class SPIClass
{
public:
    SPIClass();
    SPIClass(PinName mosi, PinName miso, PinName sclk, PinName ssel);
    SPIClass(PinName mosi, PinName miso, PinName sclk);

    void begin();
    void end();

    void beginTransaction(SPISettings settings);
    void endTransaction(void);

    uint8_t transfer(uint8_t val8);
    uint16_t transfer16(uint16_t val16);
    void transfer(void *buf, size_t count);
    void transfer(void *bufout, void *bufin, size_t count);

    void setBitOrder(BitOrder order);
    void setDataMode(uint8_t mode);
    void setClockDivider(uint32_t divider);

private:
    void config(SPISettings settings);

    SPISettings spisettings;
    bool initialized;
    spi_t _spi;
    pin_size_t _mosi;
    pin_size_t _miso;
    pin_size_t _sclk;
    pin_size_t _ssel;
    PinStatus _clockidle;
    PinStatus _clockactive;
};

extern SPIClass SPI;

#endif
