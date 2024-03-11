#ifndef CAN_COMMANDER_PROGRAM_MODULE_H
#define CAN_COMMANDER_PROGRAM_MODULE_H

#include "BaseModule.h"

class ProgramModule : public BaseModule
{
    enum FIELDS
    {
        M_PROGRAM_DEVICE_ID = 0b000,
        M_PROGRAM_DEVICE_NAME = 0b001,
        M_PROGRAM_UNIQUE_ID = 0b010,
        M_PROGRAM_WRITE_ECHO = 0b011, // should a write also be treated as request for the value? (useful to update ux/debug)
    };

public:
    // Constructor
    ProgramModule(uint8_t deviceId, uint8_t moduleId) : BaseModule(deviceId, moduleId){};

    void readRequest(uint8_t fieldId) override;
    void writeRequest(uint8_t fieldId, CanMsg *msg) override;

private:
    uint8_t _deviceName[8]; // Placeholder for device name storage
    bool writeEcho;         // Placeholder for write echo setting
    // Additional fields for device ID and other program parameters as needed
};

#endif // PROGRAM_MODULE_H
