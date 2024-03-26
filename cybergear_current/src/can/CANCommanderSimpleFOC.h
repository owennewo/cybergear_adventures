#ifndef SIMPLEFOC_CAN_H
#define SIMPLEFOC_CAN_H

#include "Arduino.h"
#include "CanMsg.h"

union CANCommanderIdentifier
{
    struct
    {
        uint16_t FIELD_ID : 3;  // FIELD ID (0-2 bit)
        uint16_t MODULE_ID : 4; // MODULE ID (3-6 bit)
        uint16_t DEVICE_ID : 4; // DEVICE ID (7-10 bit)
        uint16_t : 5;           // Unused (11-15 bit) as we are using 11bit CAN
    };
    uint16_t reg;

    void setIdentifier(uint16_t motor_id, uint16_t moduleId, uint16_t fieldId)
    {
        DEVICE_ID = motor_id;
        MODULE_ID = moduleId;
        FIELD_ID = fieldId;
    };

    static CANCommanderIdentifier from(uint32_t id)
    {
        CANCommanderIdentifier identifier;
        identifier.reg = id & 0x7FF;
        return identifier;
    }
};
#endif