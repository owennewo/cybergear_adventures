#ifndef CAN_COMMANDER_LIMITS_MODULE_H
#define CAN_COMMANDER_LIMITS_MODULE_H

#include "BaseModule.h"
#include "SimpleFOC.h" // Assume this includes necessary definitions for BLDCMotor

class LimitsModule : public BaseModule
{
    enum FIELDS
    {
        M_LIMIT_VOLTAGE_MOTOR = 0b000,
        M_LIMIT_VOLTAGE_DRIVER = 0b001,
        M_LIMIT_VOLTAGE_SENSOR_ALIGN = 0b010,
        M_LIMIT_CURRENT = 0b011,
        M_LIMIT_VELOCITY = 0b100,
    };

public:
    // Constructor that takes a pointer to BLDCMotor
    LimitsModule(uint8_t deviceId, uint8_t moduleId, BLDCMotor *motor) : BaseModule(deviceId, moduleId), _motor(motor) {}

    void readRequest(uint8_t fieldId) override;
    void writeRequest(uint8_t fieldId, CanMsg *msg) override;

private:
    BLDCMotor *_motor; // Pointer to the motor controlled by this module
};

#endif // LIMITS_MODULE_H
