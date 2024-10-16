#ifdef ARDUINO_ARCH_NRF52
#include "Hand.h"

#include "Raise2.h"
#include <cstring>

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#include "Communications.h"


namespace kaleidoscope
{
namespace device
{
namespace dygma
{
namespace raise2
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
    /* Initialize the key data */
    key_data_.all = 0;

    auto keyScanFunction = [this](Packet const &packet)
    {
        if (filterHand(packet.header.device, this_device_))
        {
            if (memcmp(key_data_.rows, packet.data, sizeof(key_data_.rows)) == 0) return;
            new_key_ = true;
            memcpy(key_data_.rows, packet.data, sizeof(key_data_.rows));
        }
    };
    Communications.callbacks.bind(HAS_KEYS, keyScanFunction);
}

void Hand::releaseAllKeys()
{
    if( key_data_.all == 0 )
    {
        /* The keys are released already */
        return;
    }

    /* Release all keys */
    key_data_.all = 0;
    new_key_ = true;
}


} // namespace raise2
} // namespace dygma
} // namespace device
} // namespace kaleidoscope
#endif
