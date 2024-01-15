## GD32 Bluepill Blinky Serial
This is using gd32 code on the gd32 bluepill that is regularly available on aliexpress:
Search term: `WeAct GD32F303CCT6 GD32F303 GD32 GD32F3 Core Board`

This board is interesting because:
 - it is similar to the chip in the cybergear `GD32F303CCT6` vs `GD32F303RET6`
 - it is much easier to test:
   - SWD interface accessible as pins (vs cybergear's solderpoints)
   - USART RX/TX accessible as pins (vs very tricky direct to chip soldering required)
 - CAN RX/TX pins accessible (but no CAN transceiver) - loopback testing possible or add external can transceiver

So the idea is get a bunch of stuff working on this bluepill and then switch to cybergear
