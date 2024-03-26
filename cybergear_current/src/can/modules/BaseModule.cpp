#include "BaseModule.h"

// BaseModule::BaseModule(uint8_t deviceID, uint8_t moduleId) : _deviceID(deviceID), _moduleId(moduleId)
// {
// }
void BaseModule::sendData(uint8_t fieldId, uint8_t *data, uint8_t dlc)
{
    uint16_t id = (_deviceId << 7) | (_moduleId << 3) | fieldId;

    CanMsg msg = CanMsg(CanStandardId(id, false), dlc, data);
    CAN.write(msg);
}

void BaseModule::sendBool(uint8_t fieldId, bool value)
{
    uint8_t data = value ? 1 : 0;
    sendData(fieldId, &data, 1);
}

void BaseModule::sendByte(uint8_t fieldId, uint8_t data)
{
    sendData(fieldId, &data, 1);
}

void BaseModule::sendSignedInt(uint8_t fieldId, int16_t data)
{
    sendData(fieldId, reinterpret_cast<uint8_t *>(&data), 2);
}

void BaseModule::sendUnsignedInt(uint8_t fieldId, uint16_t data)
{
    sendData(fieldId, reinterpret_cast<uint8_t *>(&data), 2);
}

void BaseModule::sendFloat(uint8_t fieldId, float value)
{
    uint8_t data[4];
    memcpy(data, &value, sizeof(float));
    sendData(fieldId, data, 4);
}

void BaseModule::sendFloat2x(uint8_t fieldId, float value1, float value2)
{
    uint8_t data[8];
    memcpy(data, &value1, sizeof(float));
    memcpy(data + 4, &value2, sizeof(float));
    sendData(fieldId, data, 8);
}

uint16_t BaseModule::floatToFloat16(float value)
{
    uint32_t binary = *reinterpret_cast<uint32_t *>(&value);
    uint32_t sign = (binary & 0x80000000) >> 16;
    int32_t exponent = ((binary & 0x7F800000) >> 23) - 112;
    uint32_t mantissa = (binary & 0x007FFFFF) >> 13;
    if (exponent <= 0)
    {
        return static_cast<uint16_t>(sign);
    }
    else if (exponent > 30)
    {
        return static_cast<uint16_t>(sign | 0x7BFF); // Infinity or NaN
    }
    else
    {
        return static_cast<uint16_t>(sign | (exponent << 10) | mantissa);
    }
}

void BaseModule::sendHalfFloat3x(uint8_t fieldId, float value1, float value2, float value3)
{
    uint8_t data[8];
    uint16_t temp;
    temp = floatToFloat16(value1);
    memcpy(data, &temp, sizeof(uint16_t));
    temp = floatToFloat16(value2);
    memcpy(data + 2, &temp, sizeof(uint16_t));
    temp = floatToFloat16(value3);
    memcpy(data + 4, &temp, sizeof(uint16_t));
    sendData(fieldId, data, 6);
}

float BaseModule::asFloat(uint8_t const *data)
{
    float result;
    memcpy(&result, data, sizeof(float));
    return result;
}

uint16_t BaseModule::asUnsignedInt(uint8_t const *data)
{
    return (uint16_t)(data[0] << 8 | data[1]);
}

int16_t BaseModule::asSignedInt(uint8_t const *data)
{
    return (int16_t)(data[0] << 8 | data[1]);
}

bool BaseModule::asBool(uint8_t const *data)
{
    return *data != 0;
}
