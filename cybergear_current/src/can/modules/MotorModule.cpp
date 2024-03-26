#include "MotorModule.h"

void MotorModule::readRequest(uint8_t fieldId)
{
    switch (fieldId)
    {
    case M_MOTOR_STATUS:
        sendByte(fieldId, static_cast<uint8_t>(_motor->motor_status));
        break;
    case M_MOTOR_ZERO_ELECTRIC_ANGLE:
        sendFloat(fieldId, _motor->zero_electric_angle);
        break;
    case M_MOTOR_SENSOR_DIRECTION:
        sendSignedInt(fieldId, static_cast<int16_t>(_motor->sensor_direction));
        break;
    case M_MOTOR_POLE_PAIRS:
        sendUnsignedInt(fieldId, _motor->pole_pairs);
        break;
    case M_MOTOR_PHASE_RESISTANCE:
        sendFloat(fieldId, _motor->phase_resistance);
        break;
    case M_MOTOR_PHASE_INDUCTANCE:
        sendFloat(fieldId, _motor->phase_inductance);
        break;
    case M_MOTOR_KV_RATING:
        sendFloat(fieldId, _motor->KV_rating);
        break;
    default:
        // Handle unknown field
        break;
    }
}

void MotorModule::writeRequest(uint8_t fieldId, CanMsg *msg)
{
    switch (fieldId)
    {
    case M_MOTOR_STATUS:
        _motor->motor_status = static_cast<FOCMotorStatus>(msg->data[0]);
        break;
    case M_MOTOR_ZERO_ELECTRIC_ANGLE:
        _motor->zero_electric_angle = asFloat(msg->data);
        break;
    case M_MOTOR_SENSOR_DIRECTION:
        _motor->sensor_direction = static_cast<Direction>(asUnsignedInt(msg->data));
        break;
    case M_MOTOR_POLE_PAIRS:
        _motor->pole_pairs = asUnsignedInt(msg->data);
        break;
    case M_MOTOR_PHASE_RESISTANCE:
        _motor->phase_resistance = asFloat(msg->data);
        break;
    case M_MOTOR_PHASE_INDUCTANCE:
        _motor->phase_inductance = asFloat(msg->data);
        break;
    case M_MOTOR_KV_RATING:
        _motor->KV_rating = asFloat(msg->data);
        break;
    default:
        // Handle unknown field
        break;
    }
}
