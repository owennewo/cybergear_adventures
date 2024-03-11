#include "MonitorModule.h"
#include "Arduino.h"

void MonitorModule::readRequest(uint8_t fieldId)
{
    uint8_t data[4];
    memcpy(data, &_channelIdentifiers[fieldId], sizeof(uint16_t));
    memcpy(data + 2, &_channelMillis[fieldId], sizeof(uint16_t));
    sendData(fieldId, data, 4);
}

void MonitorModule::writeRequest(uint8_t fieldId, CanMsg *msg)
{
    _channelIdentifiers[fieldId] = msg->data[0] << 8 | msg->data[1];
    _channelMillis[fieldId] = msg->data[2] << 8 | msg->data[3];
}

void MonitorModule::setChannel(uint8_t channel, uint16_t identifier, uint16_t millisPeriod)
{
    _channelLastMillis[channel] = millis();
    _channelMillis[channel] = millisPeriod;
    _channelIdentifiers[channel] = identifier;
}

bool MonitorModule::channelReady(uint8_t channel)
{
    if (_channelMillis[channel] == 0 || ((millis() - _channelLastMillis[channel]) < _channelMillis[channel]))
    {
        return false;
    }
    else
    {
        _channelLastMillis[channel] = millis();
        return true;
    }
}

uint16_t MonitorModule::getChannelIdentifier(uint8_t channel)
{
    return _channelIdentifiers[channel];
}

uint16_t MonitorModule::getChannelMillis(uint8_t channel)
{
    return _channelMillis[channel];
}