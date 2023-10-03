#ifdef ARDUINO_ARCH_NRF52
#include "Hand.h"

#include "Defy_wireless.h"
#include <cstring>

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#include "Communications.h"
#include <Kaleidoscope-IdleLEDs.h>
#include "Colormap-Defy.h"


namespace kaleidoscope
{
namespace device
{
namespace dygma
{
namespace defy_wireless
{

Hand::Hand(HandSide side) : this_device_(side)
{
}

bool inline filterHand(Communications_protocol::Devices incomingDevice,Hand::HandSide selectedDevice){
    if(selectedDevice==Hand::RIGHT){
        return incomingDevice == Communications_protocol::KEYSCANNER_DEFY_RIGHT || incomingDevice == Communications_protocol::BLE_DEFY_RIGHT ||
               incomingDevice == Communications_protocol::RF_DEFY_RIGHT;
    }else{
        return incomingDevice == Communications_protocol::KEYSCANNER_DEFY_LEFT || incomingDevice == Communications_protocol::BLE_DEFY_LEFT ||
               incomingDevice == Communications_protocol::RF_DEFY_LEFT;
    }
}

void Hand::init()
{
    auto keyScanFunction = [this](Packet const &packet)
    {
        if (filterHand(packet.header.device, this_device_))
        {
            if (memcmp(key_data_.rows, packet.data, sizeof(key_data)) == 0) return;
            new_key_ = true;
            memcpy(key_data_.rows, packet.data, sizeof(key_data));
            //                NRF_LOG_DEBUG("New key %lu",key_data_.all);
        }
    };
    Communications.callbacks.bind(HAS_KEYS, keyScanFunction);
    auto keyScanFunctionIsAlive = [this](Packet const &packet)
    {
        if(packet.data[0] != HAS_KEYS) return;
        if (filterHand(packet.header.device, this_device_))
        {
            if (memcmp(key_data_.rows, &packet.data[1], sizeof(key_data)) == 0) return;
            new_key_ = true;
            memcpy(key_data_.rows, &packet.data[1], sizeof(key_data));
            NRF_LOG_DEBUG("New key is alive %lu",key_data_.all);
        }
    };
    Communications.callbacks.bind(IS_ALIVE, keyScanFunctionIsAlive);
    auto checkConnected = [this](Packet const &packet)
    {
        if (filterHand(packet.header.device, this_device_))
        {
            NRF_LOG_DEBUG("Connected device %i",packet.header.device);
            connected_ = packet.header.device;
            if(ble_innited()){
                if(this_device_==RIGHT){
                    connected_ = BLE_DEFY_RIGHT;
                }else{
                    connected_ = BLE_DEFY_LEFT;
                }
            }
        }
    };
    Communications.callbacks.bind(CONNECTED, checkConnected);
    auto checkDisconnected = [this](Packet const &packet)
    {
        if (filterHand(packet.header.device, this_device_))
        {
            NRF_LOG_DEBUG("Disconnected device %i",packet.header.device);
            connected_ = UNKNOWN;
        }
    };
    Communications.callbacks.bind(DISCONNECTED, checkDisconnected);
}

Devices Hand::getConnectedDevice() const
{
    return connected_;
}


} // namespace defy_wireless
} // namespace dygma
} // namespace device
} // namespace kaleidoscope
#endif