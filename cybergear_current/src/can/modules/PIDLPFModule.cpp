#include "PIDLPFModule.h"

void PIDLPFModule::readRequest(uint8_t fieldId)
{
    switch (fieldId)
    {
    case P:
        sendFloat(fieldId, _pid->P);
        break;
    case I:
        sendFloat(fieldId, _pid->I);
        break;
    case D:
        sendFloat(fieldId, _pid->D);
        break;
    case LIMIT:
        sendFloat(fieldId, _pid->limit);
        break;
    case RAMP:
        sendFloat(fieldId, _pid->output_ramp);
        break;
    case LPF:
        sendFloat(fieldId, _lpf->Tf);
        break;
    default:
        // Handle unknown field
        break;
    }
}

void PIDLPFModule::writeRequest(uint8_t fieldId, CanMsg *msg)
{
    switch (fieldId)
    {
    case P:
        _pid->P = asFloat(msg->data);
        _pid->reset();
        break;
    case I:
        _pid->I = asFloat(msg->data);
        _pid->reset();
        break;
    case D:
        _pid->D = asFloat(msg->data);
        _pid->reset();
        break;
    case LIMIT:
        _pid->limit = asFloat(msg->data);
        break;
    case RAMP:
        _pid->output_ramp = asFloat(msg->data);
        break;
    case LPF:
        _lpf->Tf = asFloat(msg->data);
        break;
    default:
        // Handle unknown field
        break;
    }
}
