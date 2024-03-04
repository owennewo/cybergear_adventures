#ifndef SIMPLEFOC_CAN_H
#define SIMPLEFOC_CAN_H

#include "Arduino.h"
#include "CanMsg.h"

enum SimpleFOCModules // 4bits
{
    MODULE_CONTROL = 0b0000,
    MODULE_LIMITS = 0b0001,
    MODULE_MOTOR = 0b0010,
    MODULE_DRIVER = 0b0011,
    // PIDs
    MODULE_PIDLPF_CURRENT_Q = 0b0100, // This Module uses PIDField enum
    MODULE_PIDLPF_CURRENT_D = 0b0101, // This Module uses PIDField enum
    MODULE_PIDLPF_VELOCITY = 0b0110,  // This Module uses PIDField enum
    MODULE_PIDLPF_ANGLE = 0b0111,     // This Module uses PIDField enum
    // TELEMETRY
    MODULE_TELEMETRY = 0b1000,
    // PROGRAM
    MODULE_PROGRAM = 0b1001,
    // For SimpleFOC future use
    MODULE_RESERVED_1 = 0b1010,
    MODULE_RESERVED_2 = 0b1011,
    // For User / custom use
    MODULE_USER_1 = 0b1100,
    MODULE_USER_2 = 0b1101,
    MODULE_USER_3 = 0b1110,
    MODULE_USER_4 = 0b1111,

};

enum ModuleControl
{
    M_CONTROL_ENABLE = 0b000, // 2byte [0x00 disable || 0x01 enable, ]
    M_CONTROL_TARGET = 0b001,
    M_CONTROL_MOTION_MODE = 0b010, // 1 byte [MotionControlType enum e.g 0x00=torque control ]
    M_CONTROL_TORQUE_MODE = 0b011, // 1 byte [TorqueControlType enum e.g. 0x00=voltage]
    M_CONTROL_INIT_FOC = 0b100,
};

enum ModuleLimit
{
    M_LIMIT_VOLTAGE_MOTOR = 0b000,
    M_LIMIT_VOLTAGE_DRIVER = 0b001,
    M_LIMIT_VOLTAGE_SENSOR_ALIGN = 0b010,
    M_LIMIT_CURRENT = 0b011,
    M_LIMIT_VELOCITY = 0b100,
};

enum ModuleMotor
{
    M_MOTOR_STATUS = 0b000, // From FOCMotorStatus e.g. motor_ready = 0x04,
    M_MOTOR_ZERO_ELECTRIC_ANGLE = 0b001,
    M_MOTOR_SENSOR_DIRECTION = 0b010,
    M_MOTOR_POLE_PAIRS = 0b011,
    M_MOTOR_PHASE_RESISTANCE = 0b100,
    M_MOTOR_PHASE_INDUCTANCE = 0b101,
    M_MOTOR_KV_RATING = 0b110,

};

enum ModuleDriver
{
    M_DRIVER_VOLTAGE_POWER_SUPPLY = 0b000,
    M_DRIVER_PWM_FREQUENCY = 0b001,
    M_DRIVER_FOC_MODULATION = 0b010, // 1 byte [FOCModulationType enum e.g 0x00 SinePWM]
    M_DRIVER_MOTION_DOWNSAMPLE = 0b011,
};

enum ModuleProgram
{
    M_PROGRAM_DEVICE_ID = 0b000,
    M_PROGRAM_DEVICE_NAME = 0b001,
    M_PROGRAM_UNIQUE_ID = 0b010,
    M_PROGRAM_WRITE_ECHO = 0b011, // should a write also be treated as request for the value? (useful to update ux/debug)
};

enum ModulePIDLPF
{
    P = 0b000,     // proportional constant is multiplied by error
    I = 0b001,     // integral constant is multiplied by the sum of all previous errors
    D = 0b010,     // derivative constant is multiplied by the rate of change of the error
    LIMIT = 0b011, // saturation limit can eleviate things like integral windup
    RAMP = 0b100,  // Ramp limits how much the output can change in one iteration
    LPF = 0b101,   // Low Pass Filter time constant
};

enum ModuleTelemetry
{
    M_TELEMETRY_CHANNEL_0 = 0b000,
    M_TELEMETRY_CHANNEL_1 = 0b001,
    M_TELEMETRY_CHANNEL_2 = 0b010,
    M_TELEMETRY_CHANNEL_3 = 0b011,
    M_TELEMETRY_CONFIG_0 = 0b100,
    M_TELEMETRY_CONFIG_1 = 0b101,
    M_TELEMETRY_CONFIG_2 = 0b110,
    M_TELEMETRY_CONFIG_3 = 0b111,
};

union CANCommanderIdentifier
{
    struct
    {
        uint16_t FIELD_ID : 3;  // FIELD ID (0-2 bit)
        uint16_t MODULE_ID : 4; // MODULE ID (3-6 bit)
        uint16_t DEVICE_ID : 4; // DEVICE ID (7-10 bit)
        uint16_t : 5;           // Unused (11-15 bit) as we are using 11bit CAN
    };
    uint16_t reg;

    void setIdentifier(uint16_t motor_id, uint16_t moduleId, uint16_t fieldId)
    {
        DEVICE_ID = motor_id;
        MODULE_ID = moduleId;
        FIELD_ID = fieldId;
    };

    static CANCommanderIdentifier from(uint32_t id)
    {
        CANCommanderIdentifier identifier;
        identifier.reg = id & 0x7FF;
        return identifier;
    }
};
#endif