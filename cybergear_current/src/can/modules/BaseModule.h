#ifndef CAN_COMMANDER_BASE_MODULE_H
#define CAN_COMMANDER_BASE_MODULE_H
#include "SimpleCAN.h"

class BaseModule
{

public:
    enum MODULES : uint8_t // 4bits
    {
        MODULE_CONTROL = 0b0000,
        MODULE_LIMITS = 0b0001,
        MODULE_MOTOR = 0b0010,
        MODULE_DRIVER = 0b0011,
        // PIDs
        MODULE_PIDLPF_CURRENT_Q = 0b0100,
        MODULE_PIDLPF_CURRENT_D = 0b0101,
        MODULE_PIDLPF_VELOCITY = 0b0110,
        MODULE_PIDLPF_ANGLE = 0b0111,
        // TELEMETRY
        MODULE_METRICS = 0b1000,
        MODULE_MONITOR = 0b1001,
        // PROGRAM
        MODULE_PROGRAM = 0b1010,
        // For SimpleFOC future use
        MODULE_RESERVED_1 = 0b1011,
        MODULE_RESERVED_2 = 0b1100,
        // For User / custom use
        MODULE_USER_1 = 0b1101,
        MODULE_USER_2 = 0b1110,
        MODULE_USER_3 = 0b1111,
    };

    BaseModule(uint8_t deviceId, uint8_t moduleId) : _deviceId(deviceId), _moduleId(moduleId){};

    virtual void readRequest(uint8_t fieldId) = 0;
    virtual void writeRequest(uint8_t fieldId, CanMsg *msg) = 0;

protected:
    // CAN communication utility functions
    void sendData(uint8_t fieldId, uint8_t *data, uint8_t dlc);
    void sendBool(uint8_t fieldId, bool value);
    void sendByte(uint8_t fieldId, uint8_t data);
    void sendSignedInt(uint8_t fieldId, int16_t data);
    void sendUnsignedInt(uint8_t fieldId, uint16_t data);
    void sendFloat(uint8_t fieldId, float value);
    void sendFloat2x(uint8_t fieldId, float value1, float value2);
    void sendHalfFloat3x(uint8_t fieldId, float value1, float value2, float value3);
    uint16_t floatToFloat16(float value);

    float asFloat(uint8_t const *data);
    uint16_t asUnsignedInt(uint8_t const *data);
    int16_t asSignedInt(uint8_t const *data);
    bool asBool(uint8_t const *data);
    uint8_t _deviceId;
    uint8_t _moduleId;
};

#endif