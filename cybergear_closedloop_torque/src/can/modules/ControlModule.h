#ifndef CAN_COMMANDER_CONTROL_MODULE_H
#define CAN_COMMANDER_CONTROL_MODULE_H

#include "SimpleFOC.h"
#include "BaseModule.h"

class ControlModule : public BaseModule
{
    enum FIELDS
    {
        M_CONTROL_ENABLE = 0b000, // 2byte [0x00 disable || 0x01 enable, ]
        M_CONTROL_TARGET = 0b001,
        M_CONTROL_MOTION_MODE = 0b010, // 1 byte [MotionControlType enum e.g 0x00=torque control ]
        M_CONTROL_TORQUE_MODE = 0b011, // 1 byte [TorqueControlType enum e.g. 0x00=voltage]
        M_CONTROL_INIT_FOC = 0b100,
    };

public:
    ControlModule(uint8_t deviceId, uint8_t moduleId, BLDCMotor *motor) : BaseModule(deviceId, moduleId), _motor(motor) {}

    void readRequest(uint8_t fieldId) override;
    void writeRequest(uint8_t fieldId, CanMsg *msg) override;

private:
    BLDCMotor *_motor;
};

#endif