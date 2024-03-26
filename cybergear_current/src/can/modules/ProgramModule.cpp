#include "ProgramModule.h"
#include <cstring> // For memcpy

void ProgramModule::readRequest(uint8_t fieldId)
{
    switch (fieldId)
    {
    case M_PROGRAM_DEVICE_ID:
        sendByte(fieldId, 0x00); // Example placeholder, replace with actual logic
        break;
    case M_PROGRAM_UNIQUE_ID:
    {
        uint8_t data[8];
        // Implementation for specific architecture to retrieve the unique ID
        // Example: memcpy(data, &uniqueId, sizeof(uniqueId));
        sendData(fieldId, data, 8);
    }
    break;
    case M_PROGRAM_DEVICE_NAME:
        sendData(fieldId, _deviceName, 8);
        break;
    case M_PROGRAM_WRITE_ECHO:
        sendBool(fieldId, writeEcho);
        break;
    default:
        // Handle unknown field
        break;
    }
}

void ProgramModule::writeRequest(uint8_t fieldId, CanMsg *msg)
{
    switch (fieldId)
    {
    case M_PROGRAM_DEVICE_ID:
        // Logic to set device ID, if applicable
        break;
    case M_PROGRAM_UNIQUE_ID:
        // Typically, unique IDs are read-only and not settable, but handle if needed
        break;
    case M_PROGRAM_DEVICE_NAME:
        memcpy(_deviceName, msg->data, 8);
        break;
    case M_PROGRAM_WRITE_ECHO:
        writeEcho = asBool(msg->data);
        break;
    default:
        // Handle unknown field
        break;
    }
}
