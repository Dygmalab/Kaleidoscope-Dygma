//
// Created by Urano on 26/12/2023.
//

#include "ActionsDriver.h"

Key ActionsDriver::return_type(uint8_t tap_count, const Key *actions)
{
    switch (static_cast<Utils::TapType>(tap_count))
    {
        case Utils::TapType::None:
        {
            NRF_LOG_DEBUG("None");
            return {0xFFFF};
        }
        case Utils::TapType::Hold_Once:
        {
            NRF_LOG_DEBUG("Hold_Once");
            return actions[1];
        }
        case Utils::TapType::Tap_Once:
        {
            NRF_LOG_DEBUG("Tap_Once");
            return actions[0];
        }
        case Utils::TapType::Tap_Hold:
        {
            NRF_LOG_DEBUG("Tap_Hold");
            return actions[2];
        }
        case Utils::TapType::Tap_Twice:
        {
            NRF_LOG_DEBUG("Tap_Twice");
            return actions[3];
        }
        case Utils::TapType::Tap_Twice_Hold:
        {
            NRF_LOG_DEBUG("Tap_Twice_Hold");
            return actions[4];
        }
        default:
            NRF_LOG_DEBUG("Tap_Twice");
            return actions[3];
    }
}

bool ActionsDriver::action_handler(uint8_t tap_count, const Key *actions, const Key &key, const KeyAddr &keyAddr)
{
    //TODO: Create Key list to filter the key and run the corresponding action.
    //TODO: Add a event type HOLD or TAP in order to decide if the HOLD action has to send serveral times or only once.
    Key released_key = return_type(tap_count, actions);
    NRF_LOG_DEBUG("Key relased %i", released_key.getRaw());
    handleKeyswitchEvent(released_key, keyAddr, IS_PRESSED | INJECTED);
    kaleidoscope::Runtime.hid().keyboard().sendReport();
    return true;
}
