#include "ControlModule.h"

void ControlModule::readRequest(uint8_t fieldId)
{
    switch (fieldId)
    {
    case M_CONTROL_ENABLE:
        sendByte(fieldId, _motor->enabled);
        break;
    case M_CONTROL_TARGET:
        sendFloat(fieldId, _motor->target);
        break;
    case M_CONTROL_MOTION_MODE:
        sendByte(fieldId, _motor->controller);
        break;
    case M_CONTROL_TORQUE_MODE:
        sendByte(fieldId, _motor->torque_controller);
        break;
    case M_CONTROL_INIT_FOC:
        sendByte(fieldId, _motor->motor_status == FOCMotorStatus::motor_ready ? 1 : 0);
        break;
    default:
        // Unknown field, possibly log or handle error
        break;
    }
}

void ControlModule::writeRequest(uint8_t fieldId, CanMsg *msg)
{
    switch (fieldId)
    {
    case M_CONTROL_ENABLE:
    {
        uint8_t enable = msg->data[0];
        if (enable == 1)
        {
            _motor->enable();
        }
        else
        {
            _motor->disable();
        }
    }
    break;
    case M_CONTROL_TARGET:
        _motor->target = asFloat(msg->data);
        break;
    case M_CONTROL_MOTION_MODE:
        _motor->controller = static_cast<MotionControlType>(msg->data[0]);
        break;
    case M_CONTROL_TORQUE_MODE:
        _motor->torque_controller = static_cast<TorqueControlType>(msg->data[0]);
        break;
    case M_CONTROL_INIT_FOC:
        if (msg->data[0] == 1)
        {
            // resetting to allow initFOC to run multiple times
            _motor->motor_status = FOCMotorStatus::motor_uninitialized;
            _motor->sensor_direction = Direction::UNKNOWN;
            _motor->initFOC();
        }
        break;
    default:
        // Unknown field, possibly log or handle error
        break;
    }
}
