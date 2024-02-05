## cybergear_spi1: Reading infineon 6EDL7141 status registers

SPI1 is connected to a single device, the Infineon 6EDL7141.  This is the gatedriver that controls the MOSFETs
This device has a 24bit word length with LOTS of status and configuration registers!!

This simply reads the temperature status register.  See cybergear_spi1_driver for a more complete example.