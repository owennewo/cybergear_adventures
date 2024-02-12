## GD32 Bluepill CAN Loopback 1
This is using gd32 code on the gd32 bluepill that is regularly available on aliexpress:
Search term: `WeAct GD32F303CCT6 GD32F303 GD32 GD32F3 Core Board`

This loopback example is with code borrowed from the gd32 core example:
https://github.com/CommunityGD32Cores/gigadevice-firmware-and-docs/tree/main/GD32F1x0/GD32F1x0_Firmware_Library_V3.3.2/Examples/CAN/communication_Loopback

Loopback with interupts doesn't work, so I've adapted for a polling read which works.

Note: I'm using swd/stlink to upload (not usb)
