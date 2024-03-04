#include "CANCommander.h"
#include "CANCommanderSimpleFOC.h"
#if defined(_GD32_DEF_)

#if !defined(DEVICE_ID1)
// these are for gd32f30x
#define DEVICE_ID1 (0x1FFFF7E8U)
#define DEVICE_ID2 (0x1FFFF7ECU)
#define DEVICE_ID3 (0x1FFFF7F0U)
#endif

#endif
uint8_t *data = nullptr;

CANCommander::CANCommander()
{
}

void CANCommander::linkMotor(uint8_t deviceId, BLDCMotor *motor)
{
    _motor = motor;
    _deviceId = deviceId;
}

void sendData(uint32_t id, uint8_t *data, uint8_t dlc)
{
    CanMsg msg = CanMsg(CanStandardId(id, false), dlc, data);
    CAN.write(msg);
}

void sendBool(uint32_t id, bool value)
{
    uint8_t data = value ? 1 : 0;
    sendData(id, &data, 1);
}

void sendByte(uint32_t id, uint8_t data)
{
    sendData(id, &data, 1);
}

void sendSignedInt(uint32_t id, int16_t data)
{
    sendData(id, (uint8_t *)&data, 2);
}

void sendUnsignedInt(uint32_t id, uint16_t data)
{
    sendData(id, (uint8_t *)&data, 2);
}

void sendFloat(uint32_t id, float value)
{
    uint8_t data[4];
    memcpy(data, &value, sizeof(float));
    sendData(id, data, 4);
}

uint16_t floatToFloat16(float value)
{
    uint32_t binary = *(uint32_t *)&value;
    uint32_t sign = (binary & 0x80000000) >> 16;
    int32_t exponent = ((binary & 0x7F800000) >> 23) - 112;
    uint32_t mantissa = (binary & 0x007FFFFF) >> 13;
    if (exponent <= 0)
    {
        return sign;
    }
    else if (exponent > 30)
    {
        return sign | 0x7BFF; // Infinity or NaN
    }
    else
    {
        return sign | (exponent << 10) | mantissa;
    }
}

void sendTelemetry(uint32_t id, BLDCMotor *motor, TelemetryType type)
{
    uint8_t data[8];
    uint8_t dlc = 0;
    uint16_t temp;
    switch (type)
    {
    case TT_NONE:
        return;
    case TT_VELOCITY:
        memcpy(data, &motor->shaft_velocity, sizeof(float));
        dlc = 4;
        break;
    case TT_VELOCITY_TARGET:
        memcpy(data, &motor->target, sizeof(float));
        memcpy(data + 4, &motor->shaft_velocity, sizeof(float));
        dlc = 8;
        break;
    case TT_ANGLE:
        memcpy(data, &motor->shaft_angle, sizeof(float));
        dlc = 4;
        break;
    case TT_ANGLE_TARGET:
        memcpy(data, &motor->shaft_angle, sizeof(float));
        memcpy(data + 4, &motor->target, sizeof(float));
        dlc = 8;
        break;
    case TT_VELOCITY_ANGLE:
        memcpy(data, &motor->shaft_velocity, sizeof(float));
        memcpy(data + 4, &motor->shaft_angle, sizeof(float));
        dlc = 8;
        break;
    case TT_CURRENT_DQ:
        memcpy(data, &motor->current.d, sizeof(float));
        memcpy(data + 4, &motor->current.q, sizeof(float));
        dlc = 8;
        break;
    case TT_VOLTAGE_DQ:
        memcpy(data, &motor->voltage.d, sizeof(float));
        memcpy(data + 4, &motor->voltage.q, sizeof(float));
        dlc = 8;
        break;
    case TT_DUTYCYCLE_A:
        memcpy(data, &motor->driver->dc_a, sizeof(float));
        dlc = 4;
        break;
    case TT_DUTYCYCLE_B:
        memcpy(data, &motor->driver->dc_b, sizeof(float));
        dlc = 4;
        break;
    case TT_DUTYCYCLE_C:
        memcpy(data, &motor->driver->dc_c, sizeof(float));
        dlc = 4;
        break;
    case TT_DUTYCYCLE_ABC:
        temp = floatToFloat16(motor->driver->dc_a);
        memcpy(data, &temp, sizeof(uint16_t));
        temp = floatToFloat16(motor->driver->dc_b);
        memcpy(data + 2, &temp, sizeof(uint16_t));
        temp = floatToFloat16(motor->driver->dc_c);
        memcpy(data + 4, &temp, sizeof(uint16_t));
        dlc = 6;
        break;

    default:
        return;
    }
    CanMsg msg = CanMsg(CanStandardId(id, false), dlc, data);
    CAN.write(msg);
}

void CANCommander::run()
{
    while (CAN.available())
    {
        CanMsg rxMsg = CAN.read();
        // get the new byte:
        CANCommanderIdentifier id = CANCommanderIdentifier::from(rxMsg.id);
        uint8_t deviceId = id.DEVICE_ID;
        uint8_t moduleId = id.MODULE_ID;
        uint8_t fieldId = id.FIELD_ID;
        if (deviceId == _deviceId)
        {
            switch (moduleId)
            {
            case SimpleFOCModules::MODULE_CONTROL:
                onModuleControl(&rxMsg, fieldId);
                break;
            case SimpleFOCModules::MODULE_LIMITS:
                onModuleLimits(&rxMsg, fieldId);
                break;

            case SimpleFOCModules::MODULE_MOTOR:
                onModuleMotor(&rxMsg, fieldId);
                break;
            case SimpleFOCModules::MODULE_DRIVER:
                onModuleDriver(&rxMsg, fieldId);
                break;
            case SimpleFOCModules::MODULE_PIDLPF_CURRENT_D:
                onModulePIDLPF(&_motor->PID_current_d, &_motor->LPF_current_d, &rxMsg, fieldId);
                break;
            case SimpleFOCModules::MODULE_PIDLPF_CURRENT_Q:
                onModulePIDLPF(&_motor->PID_current_q, &_motor->LPF_current_q, &rxMsg, fieldId);
                break;
            case SimpleFOCModules::MODULE_PIDLPF_VELOCITY:
                onModulePIDLPF(&_motor->PID_velocity, &_motor->LPF_velocity, &rxMsg, fieldId);
                break;
            case SimpleFOCModules::MODULE_PIDLPF_ANGLE:
                onModulePIDLPF(&_motor->P_angle, &_motor->LPF_angle, &rxMsg, fieldId);
                break;
            case SimpleFOCModules::MODULE_TELEMETRY:
                onModuleTelemetry(&rxMsg, fieldId);
                break;
            case SimpleFOCModules::MODULE_PROGRAM:
                onModuleProgram(&rxMsg, fieldId);
                break;
            default:
                break; // unknown module
            }
        }
    }
    for (uint8_t channelId = 0; channelId < 4; channelId++)
    {
        if (_telemetryDownsample[channelId] > 0 && _telemetryCount % _telemetryDownsample[channelId] == 0)
        {
            uint32_t id = _deviceId << 7 | MODULE_TELEMETRY << 3 | channelId;
            sendTelemetry(id, _motor, (TelemetryType)_telemetryType[channelId]);
        }
    }
    _telemetryCount += 1;
}

float asFloat(uint8_t const *data)
{
    float result;
    memcpy(&result, data, sizeof(float));
    return result;
}

uint16_t asUnsignedInt(uint8_t const *data)
{
    return data[0] << 8 | data[1];
}

int16_t asSignedInt(uint8_t const *data)
{
    return (int16_t)(data[0] << 8 | data[1]);
}

bool asBool(uint8_t const *data)
{
    return *data != 0;
}

void CANCommander::onModuleControl(CanMsg *msg, uint8_t fieldId)
{
    switch (fieldId)
    {
    case ModuleControl::M_CONTROL_ENABLE:
        if (msg->isRTR())
        {
            sendByte(msg->id, _motor->enabled);
        }
        else
        {
            uint8_t enable = msg->data[0];
            if (enable == 1)
            {
                _motor->enable();
            }
            else
            {
                _motor->disable();
            }
        }
        break;
    case ModuleControl::M_CONTROL_TARGET:
        if (msg->isRTR())
        {
            sendFloat(msg->id, _motor->target);
        }
        else
        {
            _motor->target = asFloat(msg->data);
        }
        break;
    case ModuleControl::M_CONTROL_MOTION_MODE:
        if (msg->isRTR())
        {
            sendByte(msg->id, _motor->controller);
        }
        else
        {
            _motor->controller = (MotionControlType)msg->data[0];
        }
        break;

    case ModuleControl::M_CONTROL_TORQUE_MODE:
        if (msg->isRTR())
        {
            sendByte(msg->id, _motor->torque_controller);
        }
        else
        {
            _motor->torque_controller = (TorqueControlType)msg->data[0];
        }
        break;
    case ModuleControl::M_CONTROL_INIT_FOC:
        if (msg->isRTR())
        {
            sendByte(msg->id, _motor->motor_status == FOCMotorStatus::motor_ready ? 1 : 0);
        }
        else
        {
            if (msg->data[0] == 1)
            {
                // resetting to allow initFOC to run multiple times
                _motor->motor_status = FOCMotorStatus::motor_uninitialized;
                _motor->sensor_direction = Direction::UNKNOWN;
                _motor->initFOC();
            }
            else
            {
                // Do nothing?
            }
        }
        break;
    default:
        break; // unknown field for this module
    }
}

void CANCommander::onModuleDriver(CanMsg *msg, uint8_t fieldId)
{
    switch (fieldId)
    {
    case ModuleDriver::M_DRIVER_VOLTAGE_POWER_SUPPLY:
        if (msg->isRTR())
        {
            sendFloat(msg->id, _motor->driver->voltage_power_supply);
        }
        else
        {
            _motor->driver->voltage_power_supply = asFloat(msg->data);
        }
        break;
    case ModuleDriver::M_DRIVER_PWM_FREQUENCY:
        if (msg->isRTR())
        {
            sendUnsignedInt(msg->id, _motor->driver->pwm_frequency);
        }
        else
        {
            _motor->driver->pwm_frequency = asUnsignedInt(msg->data);
        }
        break;
    case ModuleDriver::M_DRIVER_FOC_MODULATION:
        if (msg->isRTR())
        {
            sendByte(msg->id, _motor->foc_modulation);
        }
        else
        {
            _motor->foc_modulation = (FOCModulationType)msg->data[0];
        }
        break;
    case ModuleDriver::M_DRIVER_MOTION_DOWNSAMPLE:
        if (msg->isRTR())
        {
            sendUnsignedInt(msg->id, _motor->motion_downsample);
        }
        else
        {
            _motor->motion_downsample = msg->data[0] << 8 | msg->data[1];
        }
        break;
    default:
        break; // unknown field for this module
    }
}

void CANCommander::onModuleLimits(CanMsg *msg, uint8_t fieldId)
{
    switch (fieldId)
    {
    case ModuleLimit::M_LIMIT_VOLTAGE_MOTOR:
        if (msg->isRTR())
        {
            sendFloat(msg->id, _motor->voltage_limit);
        }
        else
        {
            _motor->voltage_limit = asFloat(msg->data);
        }
        break;
    case ModuleLimit::M_LIMIT_VOLTAGE_DRIVER:
        if (msg->isRTR())
        {
            sendFloat(msg->id, _motor->driver->voltage_limit);
        }
        else
        {
            _motor->driver->voltage_limit = asFloat(msg->data);
        }
        break;
    case ModuleLimit::M_LIMIT_VOLTAGE_SENSOR_ALIGN:
        if (msg->isRTR())
        {
            sendFloat(msg->id, _motor->voltage_sensor_align);
        }
        else
        {
            _motor->voltage_sensor_align = asFloat(msg->data);
        }
        break;
    case ModuleLimit::M_LIMIT_CURRENT:
        if (msg->isRTR())
        {
            sendFloat(msg->id, _motor->current_limit);
        }
        else
        {
            _motor->current_limit = asFloat(msg->data);
        }
        break;
    case ModuleLimit::M_LIMIT_VELOCITY:
        if (msg->isRTR())
        {
            sendFloat(msg->id, _motor->velocity_limit);
        }
        else
        {
            _motor->velocity_limit = asFloat(msg->data);
        }
        break;
    default:
        break; // unknown field for this module
    }
}

void CANCommander::onModuleMotor(CanMsg *msg, uint8_t fieldId)
{
    switch (fieldId)
    {
    case ModuleMotor::M_MOTOR_STATUS:
        if (msg->isRTR())
        {
            sendByte(msg->id, _motor->motor_status);
        }
        else
        {
            _motor->motor_status = (FOCMotorStatus)msg->data[0];
        }
        break;
    case ModuleMotor::M_MOTOR_ZERO_ELECTRIC_ANGLE:
        if (msg->isRTR())
        {
            sendFloat(msg->id, _motor->zero_electric_angle);
        }
        else
        {
            _motor->zero_electric_angle = asFloat(msg->data);
        }
        break;
    case ModuleMotor::M_MOTOR_SENSOR_DIRECTION:
        if (msg->isRTR())
        {
            sendSignedInt(msg->id, _motor->sensor_direction);
        }
        else
        {
            _motor->sensor_direction = (Direction)asUnsignedInt(msg->data);
        }
        break;
    case ModuleMotor::M_MOTOR_POLE_PAIRS:
        if (msg->isRTR())
        {
            sendUnsignedInt(msg->id, _motor->pole_pairs);
        }
        else
        {
            _motor->pole_pairs = asUnsignedInt(msg->data);
        }
        break;
    case ModuleMotor::M_MOTOR_PHASE_RESISTANCE:
        if (msg->isRTR())
        {
            sendFloat(msg->id, _motor->phase_resistance);
        }
        else
        {
            _motor->phase_resistance = asFloat(msg->data);
        }
        break;
    case ModuleMotor::M_MOTOR_PHASE_INDUCTANCE:
        if (msg->isRTR())
        {
            sendFloat(msg->id, _motor->phase_inductance);
        }
        else
        {
            _motor->phase_inductance = asFloat(msg->data);
        }
        break;
    case ModuleMotor::M_MOTOR_KV_RATING:
        if (msg->isRTR())
        {
            sendFloat(msg->id, _motor->KV_rating);
        }
        else
        {
            _motor->KV_rating = asFloat(msg->data);
        }
        break;
    default:
        break; // unknown field for this module
    }
}

void CANCommander::onModuleTelemetry(CanMsg *msg, uint8_t fieldId)
{
    if (fieldId < 4)
    {
        // OUT
        if (msg->isRTR())
        {

            TelemetryType type = _telemetryType[fieldId];
            sendTelemetry(msg->id, _motor, type);
        }
        else
        {
            ; // READ ONLY.
            // To change channel type, use ModuleProgram::M_PROGRAM_TELEMETRY_TYPE (offerring read/write telementry config)
        }
    }
    else
    {
        // CTL
        if (msg->isRTR())
        {
            TelemetryType type = _telemetryType[fieldId - 4];
            uint16_t downsample = _telemetryDownsample[fieldId - 4];
            uint8_t data[4];
            memcpy(data, &type, sizeof(uint16_t));
            memcpy(data + 2, &type, sizeof(uint16_t));
            sendData(msg->id, data, 4);
        }
        else
        {
            _telemetryType[fieldId - 4] = (TelemetryType)(data[0] << 8 & data[1]);
            _telemetryDownsample[fieldId - 4] = data[2] << 8 & data[3];
        }
    }
}

void CANCommander::onModulePIDLPF(PIDController *pid, LowPassFilter *lpf, CanMsg *msg, uint8_t fieldId)
{
    switch (fieldId)
    {
    case ModulePIDLPF::P:
        if (msg->isRTR())
        {
            sendFloat(msg->id, pid->P);
        }
        else
        {
            pid->P = asFloat(msg->data);
        }
        break;
    case ModulePIDLPF::I:
        if (msg->isRTR())
        {
            sendFloat(msg->id, pid->I);
        }
        else
        {
            pid->I = asFloat(msg->data);
        }
        break;
    case ModulePIDLPF::D:
        if (msg->isRTR())
        {
            sendFloat(msg->id, pid->D);
        }
        else
        {
            pid->D = asFloat(msg->data);
        }
        break;
    case ModulePIDLPF::LIMIT:
        if (msg->isRTR())
        {
            sendFloat(msg->id, pid->limit);
        }
        else
        {
            pid->limit = asFloat(msg->data);
        }
        break;

    case ModulePIDLPF::RAMP:
        if (msg->isRTR())
        {
            sendFloat(msg->id, pid->output_ramp);
        }
        else
        {
            pid->output_ramp = asFloat(msg->data);
        }
        break;
    case ModulePIDLPF::LPF:
        if (msg->isRTR())
        {
            sendFloat(msg->id, lpf->Tf);
        }
        else
        {
            lpf->Tf = asFloat(msg->data);
        }
        break;
    default:
        break; // unknown field for this module
    }
}

void CANCommander::onModuleProgram(CanMsg *msg, uint8_t fieldId)
{
    switch (fieldId)
    {

    case ModuleProgram::M_PROGRAM_DEVICE_ID:
        if (msg->isRTR())
        {
            sendByte(msg->id, 0x00); // TODO: get device id.  Need a way to store device permanently
        }
        else
        {
            // TODO: Read device id
        }
        break;
    case ModuleProgram::M_PROGRAM_UNIQUE_ID:
        if (msg->isRTR())
        {
            uint8_t data[8];
#if defined(ARDUINO_ARCH_STM32)
            uint32_t uid0 = HAL_GetUIDw0();
            uint32_t uid1 = HAL_GetUIDw1();
            // uint32_t uid2 = HAL_GetUIDw1();

            memcpy(data, &uid0, sizeof(uint32_t));
            memcpy(data + 4, &uid1, sizeof(uint32_t));
#elif defined(_GD32_DEF_)
            uint32_t uid0 = *(uint32_t *)DEVICE_ID1;
            uint32_t uid1 = *(uint32_t *)DEVICE_ID2;
            // uint32_t uid2 = *(uint32_t*)DEVICE_ID3;

            memcpy(data, &uid0, sizeof(uint32_t));
            memcpy(data + 4, &uid1, sizeof(uint32_t));
#endif
            sendData(msg->id, data, 8); // TODO: get device id.  Need a way to store device permanently
        }
        else
        {
            // TODO: Read device id
        }
        break;
    case ModuleProgram::M_PROGRAM_DEVICE_NAME:
        if (msg->isRTR())
        {
            sendData(msg->id, _deviceName, 8);
        }
        else
        {
            memcpy(_deviceName, msg->data, 8);
        }
        break;
    case ModuleProgram::M_PROGRAM_WRITE_ECHO:
        if (msg->isRTR())
        {
            sendBool(msg->id, writeEcho);
        }
        else
        {
            writeEcho = asBool(msg->data);
        }
        break;
    default:
        break; // unknown field for this module
    }
}
