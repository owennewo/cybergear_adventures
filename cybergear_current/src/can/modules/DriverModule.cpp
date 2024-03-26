#include "DriverModule.h"

void DriverModule::readRequest(uint8_t fieldId)
{
    switch (fieldId)
    {
    case M_DRIVER_VOLTAGE_POWER_SUPPLY:
        sendFloat(fieldId, _motor->driver->voltage_power_supply);
        break;
    case M_DRIVER_PWM_FREQUENCY:
        sendUnsignedInt(fieldId, _motor->driver->pwm_frequency);
        break;
    case M_DRIVER_FOC_MODULATION:
        sendByte(fieldId, static_cast<uint8_t>(_motor->foc_modulation));
        break;
    case M_DRIVER_MOTION_DOWNSAMPLE:
        sendUnsignedInt(fieldId, _motor->motion_downsample);
        break;
    default:
        // Handle unknown field
        break;
    }
}

void DriverModule::writeRequest(uint8_t fieldId, CanMsg *msg)
{
    switch (fieldId)
    {
    case M_DRIVER_VOLTAGE_POWER_SUPPLY:
        _motor->driver->voltage_power_supply = asFloat(msg->data);
        break;
    case M_DRIVER_PWM_FREQUENCY:
        _motor->driver->pwm_frequency = asUnsignedInt(msg->data);
        break;
    case M_DRIVER_FOC_MODULATION:
        _motor->foc_modulation = static_cast<FOCModulationType>(msg->data[0]);
        break;
    case M_DRIVER_MOTION_DOWNSAMPLE:
        _motor->motion_downsample = asUnsignedInt(msg->data);
        break;
    default:
        // Handle unknown field
        break;
    }
}
