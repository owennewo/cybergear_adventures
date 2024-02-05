## cybergear_spi2: AS5047p Magnetic sensor

This sample demonstrates the SPI2 peripheral which has a single AS50479 magnetic sensor connected to it.
It measures the angle (motor shaft position) and later will be used for closed loop motor control.

There is no usart on cybergear so use a debugger or stm monitor to check the angle.  Or you could listen to the CAN bus to see the angle.
