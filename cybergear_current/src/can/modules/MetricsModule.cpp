#include "MetricsModule.h"

void MetricsModule::readRequest(uint8_t fieldId)
{
    switch (fieldId)
    {
    case M_METRICS_VELOCITY_ANGLE:
        sendFloat2x(fieldId, _motor->shaft_velocity, _motor->shaft_angle);
        break;
    case M_METRICS_CURRENT_DQ:
        sendFloat2x(fieldId, _motor->current.d, _motor->current.q);
        break;
    case M_METRICS_VOLTAGE_DQ:
        sendFloat2x(fieldId, _motor->voltage.d, _motor->voltage.q);
        break;
    case M_METRICS_DUTYCYCLE_ABC:
        sendHalfFloat3x(fieldId, _motor->driver->dc_a, _motor->driver->dc_b, _motor->driver->dc_c);
        break;
    default:
        // Handle unknown field
        break;
    }
}

void MetricsModule::writeRequest(uint8_t fieldId, CanMsg *msg)
{
    // READ ONLY, nothing to do here
}
