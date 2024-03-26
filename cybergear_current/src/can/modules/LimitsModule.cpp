#include "LimitsModule.h"

void LimitsModule::readRequest(uint8_t fieldId)
{
    switch (fieldId)
    {
    case M_LIMIT_VOLTAGE_MOTOR:
        sendFloat(fieldId, _motor->voltage_limit);
        break;
    case M_LIMIT_VOLTAGE_DRIVER:
        sendFloat(fieldId, _motor->driver->voltage_limit);
        break;
    case M_LIMIT_VOLTAGE_SENSOR_ALIGN:
        sendFloat(fieldId, _motor->voltage_sensor_align);
        break;
    case M_LIMIT_CURRENT:
        sendFloat(fieldId, _motor->current_limit);
        break;
    case M_LIMIT_VELOCITY:
        sendFloat(fieldId, _motor->velocity_limit);
        break;
    default:
        // Handle unknown field
        break;
    }
}

void LimitsModule::writeRequest(uint8_t fieldId, CanMsg *msg)
{
    switch (fieldId)
    {
    case M_LIMIT_VOLTAGE_MOTOR:
        _motor->voltage_limit = asFloat(msg->data);
        break;
    case M_LIMIT_VOLTAGE_DRIVER:
        _motor->driver->voltage_limit = asFloat(msg->data);
        break;
    case M_LIMIT_VOLTAGE_SENSOR_ALIGN:
        _motor->voltage_sensor_align = asFloat(msg->data);
        break;
    case M_LIMIT_CURRENT:
        _motor->current_limit = asFloat(msg->data);
        break;
    case M_LIMIT_VELOCITY:
        _motor->velocity_limit = asFloat(msg->data);
        break;
    default:
        // Handle unknown field
        break;
    }
}
