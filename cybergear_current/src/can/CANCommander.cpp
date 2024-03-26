#include "SimpleFOC.h"
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

CANCommander::CANCommander()
{
}

void CANCommander::linkMotor(uint8_t deviceId, BLDCMotor *motor)
{
    _deviceId = deviceId;
    _modules[BaseModule::MODULE_CONTROL] = new ControlModule(deviceId, BaseModule::MODULE_CONTROL, motor);
    _modules[BaseModule::MODULE_LIMITS] = new LimitsModule(deviceId, BaseModule::MODULE_LIMITS, motor);
    _modules[BaseModule::MODULE_MOTOR] = new MotorModule(deviceId, BaseModule::MODULE_MOTOR, motor);
    _modules[BaseModule::MODULE_DRIVER] = new DriverModule(deviceId, BaseModule::MODULE_DRIVER, motor);
    _modules[BaseModule::MODULE_PIDLPF_CURRENT_Q] = new PIDLPFModule(deviceId, BaseModule::MODULE_PIDLPF_CURRENT_Q, &motor->PID_current_q, &motor->LPF_current_q);
    _modules[BaseModule::MODULE_PIDLPF_CURRENT_D] = new PIDLPFModule(deviceId, BaseModule::MODULE_PIDLPF_CURRENT_D, &motor->PID_current_d, &motor->LPF_current_d);
    _modules[BaseModule::MODULE_PIDLPF_VELOCITY] = new PIDLPFModule(deviceId, BaseModule::MODULE_PIDLPF_VELOCITY, &motor->PID_velocity, &motor->LPF_velocity);
    _modules[BaseModule::MODULE_PIDLPF_ANGLE] = new PIDLPFModule(deviceId, BaseModule::MODULE_PIDLPF_ANGLE, &motor->P_angle, &motor->LPF_angle);
    _modules[BaseModule::MODULE_METRICS] = new MetricsModule(deviceId, BaseModule::MODULE_METRICS, motor);
    _monitorModule = new MonitorModule(deviceId, BaseModule::MODULE_MONITOR, motor);
    _modules[BaseModule::MODULE_MONITOR] = _monitorModule;
    _modules[BaseModule::MODULE_PROGRAM] = new ProgramModule(deviceId, BaseModule::MODULE_PROGRAM);
}

void CANCommander::run()
{
    while (CAN.available())
    {
        CanMsg rxMsg = CAN.read();
        CANCommanderIdentifier id = CANCommanderIdentifier::from(rxMsg.id);
        bool isWrite = !rxMsg.isRTR();
        bool isRead = rxMsg.isRTR() || _writeEcho;
        if (id.DEVICE_ID == _deviceId)
        {
            if (_modules[id.MODULE_ID] != nullptr)
            {
                if (isWrite)
                {
                    _modules[id.MODULE_ID]->writeRequest(id.FIELD_ID, &rxMsg);
                }
                if (isRead)
                {
                    _modules[id.MODULE_ID]->readRequest(id.FIELD_ID);
                }
                break;
            }
        }
    }

    for (uint8_t channelId = 0; channelId < 8; channelId++)
    {
        if (_monitorModule->channelReady(channelId))
        {
            uint16_t identifier = _monitorModule->getChannelIdentifier(channelId);
            CANCommanderIdentifier id = CANCommanderIdentifier::from(identifier);
            _modules[id.MODULE_ID]->readRequest(id.FIELD_ID);
        }
    }
}

void CANCommander::addModule(uint8_t moduleId, BaseModule *module)
{
    _modules[moduleId] = module;
}

void CANCommander::monitor(uint8_t channel, uint8_t moduleId, uint8_t fieldId, uint16_t millis)
{
    uint16_t identifier = _deviceId << 7 | moduleId << 3 | fieldId;
    _monitorModule->setChannel(channel, identifier, millis);
}

void CANCommander::sendFloat(uint32_t id, float value)
{
    uint8_t data[4];
    memcpy(data, &value, sizeof(float));
    CanMsg msg = CanMsg(CanStandardId(id, false), 4, data);
    CAN.write(msg);
}
