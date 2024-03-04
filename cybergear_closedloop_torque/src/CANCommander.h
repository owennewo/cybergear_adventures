#ifndef CAN_COMMANDER_H
#define CAN_COMMANDER_H

#include "Arduino.h"
#include "SimpleFOC.h"
#include "SimpleCAN.h"

typedef void (*CANCommandCallback)(CanMsg *msg);

enum TelemetryType : uint16_t
{
    TT_NONE = 0x00,            // No data
    TT_VELOCITY = 0x01,        // float
    TT_VELOCITY_TARGET = 0x02, // float[2]
    TT_ANGLE = 0x03,           // float
    TT_ANGLE_TARGET = 0x04,    // float[2]
    TT_VELOCITY_ANGLE = 0x05,  // float[2]
    TT_CURRENT_DQ = 0x06,      // float[2]
    TT_VOLTAGE_DQ = 0x07,      // float[2]
    TT_DUTYCYCLE_A = 0x08,     // float
    TT_DUTYCYCLE_B = 0x09,     // float
    TT_DUTYCYCLE_C = 0x0A,     // float
    TT_DUTYCYCLE_ABC = 0x10,   // 1/2 presicion float[3] - 6 bytes
};

class CANCommander
{
public:
    CANCommander();

    void run();

    void linkMotor(uint8_t deviceId, BLDCMotor *motor);

private:
    void onModuleControl(CanMsg *msg, uint8_t fieldId);
    void onModuleLimits(CanMsg *msg, uint8_t fieldId);
    void onModuleMotor(CanMsg *msg, uint8_t fieldId);
    void onModuleDriver(CanMsg *msg, uint8_t fieldId);
    void onModuleSensor(CanMsg *msg, uint8_t fieldId);
    void onModulePIDLPF(PIDController *pid, LowPassFilter *lpf, CanMsg *msg, uint8_t fieldId);
    void onModuleTelemetry(CanMsg *msg, uint8_t fieldId);
    void onModuleProgram(CanMsg *msg, uint8_t fieldId);

    bool writeEcho = true;
    BLDCMotor *_motor;
    uint8_t _deviceId;
    byte _deviceName[8] = {'c', 'h', 'a', 'n', 'g', 'e', 'm', 'e'};
    TelemetryType _telemetryType[4] = {TT_VELOCITY, TT_ANGLE, TT_NONE, TT_NONE};
    uint16_t _telemetryDownsample[4] = {uint16_t(1 << 14), uint16_t(1 << 14), uint16_t(0), uint16_t(0)};
    uint16_t _telemetryCount = 0;
};

#endif