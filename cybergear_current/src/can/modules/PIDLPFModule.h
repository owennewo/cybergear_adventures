#ifndef CAN_COMMANDER_PID_LPF_MODULE_H
#define CAN_COMMANDER_PID_LPF_MODULE_H

#include "BaseModule.h"
#include "SimpleFOC.h" // Assume necessary definitions for PIDController and LowPassFilter

class PIDLPFModule : public BaseModule
{
public:
    enum ModulePIDLPF
    {
        P = 0b000,     // proportional constant is multiplied by error
        I = 0b001,     // integral constant is multiplied by the sum of all previous errors
        D = 0b010,     // derivative constant is multiplied by the rate of change of the error
        LIMIT = 0b011, // saturation limit can eleviate things like integral windup
        RAMP = 0b100,  // Ramp limits how much the output can change in one iteration
        LPF = 0b101,   // Low Pass Filter time constant
    };

    PIDLPFModule(uint8_t deviceId, uint8_t moduleId, PIDController *pid, LowPassFilter *lpf) : BaseModule(deviceId, moduleId),
                                                                                               _pid(pid),
                                                                                               _lpf(lpf){};

    void readRequest(uint8_t fieldId) override;
    void writeRequest(uint8_t fieldId, CanMsg *msg) override;

private:
    PIDController *_pid; // Pointer to the PID controller
    LowPassFilter *_lpf; // Pointer to the Low Pass Filter
};

#endif // PID_LPF_MODULE_H
