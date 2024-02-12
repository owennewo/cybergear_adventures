## GD32 Bluepill CAN Loopback STM
This is using stm32duino and therefore using STM core and HAL.

This project has a couple of problems:
 - legal problem.  The gd32 is configured to use stm libs.  This is against stm's terms.  Should be using gd32 core
 - the can init times out

 I suspect with more work it could be made to work, but I'm not contunuining on the stm32duino route.

Note: I'm using swd/stlink to upload (not usb)
