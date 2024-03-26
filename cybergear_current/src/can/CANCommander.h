#ifndef CAN_COMMANDER_H
#define CAN_COMMANDER_H

#include "Arduino.h"
#include "SimpleFOC.h"
#include "SimpleCAN.h"
#include "modules/BaseModule.h"
#include "modules/ControlModule.h"
#include "modules/LimitsModule.h"
#include "modules/MotorModule.h"
#include "modules/DriverModule.h"
#include "modules/PIDLPFModule.h"
#include "modules/MonitorModule.h"
#include "modules/MetricsModule.h"
#include "modules/ProgramModule.h"

typedef void (*CANCommandCallback)(CanMsg *msg);

class CANCommander
{
public:
    CANCommander();

    void run();
    void linkMotor(uint8_t deviceId, BLDCMotor *motor);

    // addModule for additional custom modules
    void addModule(uint8_t moduleId, BaseModule *module);
    void monitor(uint8_t channel, uint8_t moduleId, uint8_t fieldId, uint16_t millis);

    // TODO: REMOVE this debug function
    void sendFloat(uint32_t id, float value);

private:
    bool _writeEcho = true;
    uint8_t _deviceId;
    BaseModule *_modules[16] = {nullptr};
    MonitorModule *_monitorModule = nullptr;
};

#endif