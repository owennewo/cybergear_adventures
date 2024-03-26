#ifndef CAN_COMMANDER_MONITOR_MODULE_H
#define CAN_COMMANDER_MONITOR_MODULE_H

#include "BaseModule.h"
#include "SimpleFOC.h"

class MonitorModule : public BaseModule
{
public:
    enum FIELDS
    {
        M_MONITOR_CHANNEL_0 = 0b000,
        M_MONITOR_CHANNEL_1 = 0b001,
        M_MONITOR_CHANNEL_2 = 0b010,
        M_MONITOR_CHANNEL_3 = 0b011,
        M_MONITOR_CHANNEL_4 = 0b100,
        M_MONITOR_CHANNEL_5 = 0b101,
        M_MONITOR_CHANNEL_6 = 0b110,
        M_MONITOR_CHANNEL_7 = 0b111,
    };

    MonitorModule(uint8_t deviceId, uint8_t moduleId, BLDCMotor *motor) : BaseModule(deviceId, moduleId), _motor(motor) {}

    void readRequest(uint8_t fieldId) override;
    void writeRequest(uint8_t fieldId, CanMsg *msg) override;
    void run(uint8_t deviceId);
    void setChannel(uint8_t channel, uint16_t identifier, uint16_t millis);
    uint16_t getChannelIdentifier(uint8_t channel);
    uint16_t getChannelMillis(uint8_t channel);
    bool channelReady(uint8_t channel);

private:
    BLDCMotor *_motor;                                  // Pointer to the motor controlled by this module
    uint16_t _channelIdentifiers[8];                    //
    uint16_t _channelMillis[8]{0, 0, 0, 0, 0, 0, 0, 0}; // millis intervals between sample
    uint32_t _channelLastMillis[0];                     // last
};

#endif // MONITOR_MODULE_H
