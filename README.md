# cybergear_adventures
A set of arduino projects where I explore cybergear actuator using opensource code.

 * bluepill_* projects use a similar gd32 chip which is easier to debug.  You can probably ignore these.
 * cybergear_blinky_serial - it blinks.  It won't print serial unless you bravely solder rx/tx directly to pins 16 and 17
 * cybergear_simplcan_loopback - this uses my simplecan (cross platform) library to talk using CAN0.  Loopback means the can messages echo back from tx to rx
 * cybergear_simplecan_normal.  Simple can messages are sent out.  Useful to have a usb/can dongle to test 
 * cybergear_spi1 - simple read on temp register of the infineon gate driver chip
 * cybergear_spi1_bitbang - I tried a software bitbanging approach to help work out what was going on, ignore
 * cybergear_spi1_driver - this use a driver that I'm adding to simplefoc for the infineon - it supports all status/congiguration registers
 * cybergear_spi2 - this talks to the as5047p magnetic sensor and reads motor shaft angle

What's left to do
 * PWM configuration
 * integration with simplefoc
 * current control 
 * proper CAN API similar to simpefoc commander (which is a usart api)
