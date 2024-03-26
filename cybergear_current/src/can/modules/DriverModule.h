#ifndef CAN_COMMANDER_DRIVER_MODULE_H
#define CAN_COMMANDER_DRIVER_MODULE_H

#include "BaseModule.h"
#include "SimpleFOC.h" // Assume this includes necessary definitions for BLDCMotor and driver

class DriverModule : public BaseModule
{
public:
    enum FIELDS
    {
        M_DRIVER_VOLTAGE_POWER_SUPPLY = 0b000,
        M_DRIVER_PWM_FREQUENCY = 0b001,
        M_DRIVER_FOC_MODULATION = 0b010, // 1 byte [FOCModulationType enum e.g 0x00 SinePWM]
        M_DRIVER_MOTION_DOWNSAMPLE = 0b011,
    };

    // Constructor that takes a pointer to BLDCMotor, assuming motor includes a driver
    DriverModule(uint8_t deviceId, uint8_t moduleId, BLDCMotor *motor) : BaseModule(deviceId, moduleId), _motor(motor) {}

    void readRequest(uint8_t fieldId) override;
    void writeRequest(uint8_t fieldId, CanMsg *msg) override;

private:
    BLDCMotor *_motor; // Pointer to the motor controlled by this module
};

#endif // DRIVER_MODULE_H
