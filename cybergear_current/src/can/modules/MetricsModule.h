#ifndef CAN_COMMANDER_METRIC_MODULE_H
#define CAN_COMMANDER_METRIC_MODULE_H

#include "BaseModule.h"
#include "SimpleFOC.h"

class MetricsModule : public BaseModule
{
public:
    enum FIELDS : uint8_t
    {
        M_METRICS_VELOCITY_ANGLE = 0b000,
        M_METRICS_CURRENT_DQ = 0b001,
        M_METRICS_VOLTAGE_DQ = 0b010,
        M_METRICS_DUTYCYCLE_ABC = 0b011,
    };

    MetricsModule(uint8_t deviceId, uint8_t moduleId, BLDCMotor *motor) : BaseModule(deviceId, moduleId), _motor(motor) {}

    void readRequest(uint8_t fieldId) override;
    void writeRequest(uint8_t fieldId, CanMsg *msg) override;

private:
    BLDCMotor *_motor;
};

#endif // CAN_COMMANDER_METRIC_MODULE_H
