
#include "ActionsDriver.h"

Key ActionsDriver::return_type(uint8_t tap_count, const Key *actions, bool repeat_on_hold)
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
        check_if_key_is_repeateable(actions[1]);
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
        check_if_key_is_repeateable(actions[2]);
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
        check_if_key_is_repeateable(actions[4]);
        return actions[4];
    }
    default:
    {
        NRF_LOG_DEBUG("Tap_Trice or more");
        return actions[3];
    }
    }
}

bool ActionsDriver::action_handler(uint8_t tap_count, const Key *actions, const Key &key, const KeyAddr &keyAddr, bool repeat_on_hold)
{
    // TODO: Create Key list to filter the key and run the corresponding action.
    // TODO: Add a event type HOLD or TAP in order to decide if the HOLD action has to send serveral times or only once.

    Key released_key = return_type(tap_count, actions, repeat_on_hold);

    NRF_LOG_DEBUG("Key relased %i", released_key.getRaw());
    handleKeyswitchEvent(released_key, keyAddr, IS_PRESSED | INJECTED);

    kaleidoscope::Runtime.hid().keyboard().sendReport();
    return true;
}

bool ActionsDriver::isOnlyModifier(Key key)
{
    uint16_t raw = key.getRaw();
    uint8_t modif = (raw & 0xFF00) >> 8;
    uint8_t key_id = raw & 0x00FF;

    return (key_id == Key_NoKey.getRaw() && modif != 0);
}

bool ActionsDriver::check_if_key_is_repeateable(const Key &key)
{
    // Check if the key in the key is a modifier key , combination of modifier keys ONLY or a layer shift.
    // If we found any other key, we return false.

    if (isOnlyModifier(key))
    {
        NRF_LOG_DEBUG("Key %i is a modifier key only", key.getRaw());
        return true;
    }

    Utils::KeyRanges ranges_t = static_cast<Utils::KeyRanges>(find_key_type(key.getRaw()));

    switch (ranges_t)
    {
    case Utils::KeyRanges::LAYER_LOCK:
    case Utils::KeyRanges::DYNAMIC_MACRO:
    case Utils::KeyRanges::ALPHA_WITH_MODIFIERS:
    {
        NRF_LOG_DEBUG("Key %i is not reapeatable", key.getRaw());
        return false;
    }
    break;

    case Utils::KeyRanges::LAYER_SHIFT:
    {
        NRF_LOG_DEBUG("Key %i is reapeatable", key.getRaw());
        return true;
    }
    break;

    default:
    {
        NRF_LOG_DEBUG("Key %i is not reapeatable", key.getRaw());
        return false;
    }
    break;
    }
}

bool ActionsDriver::check_if_sk_interruptable(const Key &Action)
{
    auto ranges_t = static_cast<Utils::KeyRanges>(find_key_type(Action.getRaw()));
    switch (ranges_t)
    {
    case Utils::KeyRanges::LAYER_LOCK:
    case Utils::KeyRanges::DYNAMIC_MACRO:
    case Utils::KeyRanges::LAYER_SHIFT:
    case Utils::KeyRanges::ALPHA_WITH_MODIFIERS:
    {
        NRF_LOG_DEBUG("superkey  is NOT interruptable ");
        return false;
    }
    break;

    default:
    {
        NRF_LOG_DEBUG("superkey  is interruptable ");
        return true;
    }
    break;
    }
}

uint16_t ActionsDriver::find_key_type(uint16_t value)
{
    int start = 0;
    int end = Utils::numRanges - 1;

    while (start <= end)
    {
        int mid = start + (end - start) / 2;

        if (value >= Utils::ranges[mid].start && value <= Utils::ranges[mid].end)
        {
            return mid; // Found the range
        }
        else if (value < Utils::ranges[mid].start)
        {
            end = mid - 1; // Search in the left half
        }
        else
        {
            start = mid + 1; // Search in the right half
        }
    }
    return -1;
}