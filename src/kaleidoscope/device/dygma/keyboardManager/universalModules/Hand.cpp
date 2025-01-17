/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::Raise -- Kaleidoscope device plugin for Dygma Raise
 * Copyright (C) 2017-2019  Keyboard.io, Inc
 * Copyright (C) 2017-2020  Dygma Lab S.L.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef ARDUINO_ARCH_NRF52
#include "Hand.h"

#include "KeyboardManager.h"
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
namespace dygma_keyboards
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


} // namespace dygma_keyboards
} // namespace dygma
} // namespace device
} // namespace kaleidoscope
#endif
