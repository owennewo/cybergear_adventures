#ifndef CAN_COMMANDER_MOTOR_MODULE_H
#define CAN_COMMANDER_MOTOR_MODULE_H

#include "BaseModule.h"
#include "SimpleFOC.h" // Assume this includes necessary definitions for BLDCMotor

class MotorModule : public BaseModule
{
    enum FIELDS
    {
        M_MOTOR_STATUS = 0b000, // From FOCMotorStatus e.g. motor_ready = 0x04,
        M_MOTOR_ZERO_ELECTRIC_ANGLE = 0b001,
        M_MOTOR_SENSOR_DIRECTION = 0b010,
        M_MOTOR_POLE_PAIRS = 0b011,
        M_MOTOR_PHASE_RESISTANCE = 0b100,
        M_MOTOR_PHASE_INDUCTANCE = 0b101,
        M_MOTOR_KV_RATING = 0b110,
    };

public:
    // Constructor that takes a pointer to BLDCMotor
    MotorModule(uint8_t deviceId, uint8_t moduleId, BLDCMotor *motor) : BaseModule(deviceId, moduleId), _motor(motor) {}

    void readRequest(uint8_t fieldId) override;
    void writeRequest(uint8_t fieldId, CanMsg *msg) override;

private:
    BLDCMotor *_motor; // Pointer to the motor controlled by this module
};

#endif // MOTOR_MODULE_H
