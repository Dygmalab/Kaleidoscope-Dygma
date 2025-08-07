
#include "ActionsDriver.h"

Utils::ExtendedActions ActionsDriver::return_type(uint8_t tap_count, const Key *actions)
{
    Utils::ExtendedActions sk_key_action = {0xFFFF, false, false};
    switch (static_cast<Utils::TapType>(tap_count))
    {
    case Utils::TapType::None:
    {
        // NRF_LOG_DEBUG("None");
        sk_key_action.key = 0xFFFF;
        sk_key_action.key_is_interruptable = false;
        return {0xFFFF};
    }
    break;

    case Utils::TapType::Tap_Once:
    {
        // NRF_LOG_DEBUG("Tap_Once");
        sk_key_action.key = actions[0];
        sk_key_action.key_is_interruptable = key_can_interrupt(actions[0]);
    }
    break;

    case Utils::TapType::Hold_Once:
    {
        // NRF_LOG_DEBUG("Hold_Once");
        sk_key_action.key = actions[1];
        sk_key_action.key_is_interruptable = key_can_interrupt(actions[1]);
    }
    break;

    case Utils::TapType::Tap_Hold:
    {
        // NRF_LOG_DEBUG("Tap_Hold");
        sk_key_action.key = actions[2];
        sk_key_action.key_is_interruptable = key_can_interrupt(actions[2]);
    }
    break;

    case Utils::TapType::Tap_Twice:
    {
        // NRF_LOG_DEBUG("Tap_Twice");
        sk_key_action.key = actions[3];
        sk_key_action.key_is_interruptable = key_can_interrupt(actions[3]);
        if(sk_key_action.key == 1)
        {
            // If the tap twice action is not set, we will release the tap action twice.
            sk_key_action.key = actions[0];
            sk_key_action.release_two_keys = true;
        }
        else
        {
            sk_key_action.release_two_keys = false;
        }
    }
    break;

    case Utils::TapType::Tap_Twice_Hold:
    {
        // NRF_LOG_DEBUG("Tap_Twice_Hold");
        sk_key_action.key = actions[4];
        sk_key_action.key_is_interruptable = key_can_interrupt(actions[4]);
    }
    break;

    default:
    {
        // NRF_LOG_DEBUG("Tap_Trice or more");
        sk_key_action.key = actions[3];
        sk_key_action.key_is_interruptable = key_can_interrupt(actions[3]);
        if(sk_key_action.key == 1)
        {
            // If the tap twice action is not set, we will release the tap action twice.
            sk_key_action.key = actions[0];
            sk_key_action.release_two_keys = true;
        }
        else
        {
            sk_key_action.release_two_keys = false;
        }
    }
    break;
    }
    return sk_key_action;
}

void logKeyModifiers(Key key)
{
    uint8_t flags = key.getFlags();

    if (flags & CTRL_HELD)
        NRF_LOG_DEBUG("CTRL held");
    if (flags & LALT_HELD)
        NRF_LOG_DEBUG("Left ALT held");
    if (flags & RALT_HELD)
        NRF_LOG_DEBUG("Right ALT held");
    if (flags & SHIFT_HELD)
        NRF_LOG_DEBUG("SHIFT held");
    if (flags & GUI_HELD)
        NRF_LOG_DEBUG("GUI/Win held");
    if (flags == 0)
        NRF_LOG_DEBUG("No modifiers");
}

bool ActionsDriver::action_handler(uint8_t tap_count, const Key *actions, const Key &key, const KeyAddr &keyAddr)
{
    bool result = false;
    // TODO: Create Key list to filter the key and run the corresponding action.
    // TODO: Add a event type HOLD or TAP in order to decide if the HOLD action has to send serveral times or only once.

    Utils::ExtendedActions returned_key = return_type(tap_count, actions);

        NRF_LOG_DEBUG("Key released raw=%i flags=%i",
                      returned_key.key.getRaw(),
                      returned_key.key.getFlags());

    if (returned_key.key_is_interruptable)
    {
        result = true;
    }

    if(returned_key.release_two_keys)
    {
        send_key_twice(returned_key.key, keyAddr);
    }
    else
    {
        handleKeyswitchEvent(returned_key.key, keyAddr, IS_PRESSED | INJECTED);
    }
    // logKeyModifiers(returned_key.key);
    return result;
}

bool ActionsDriver::isOnlyModifier(Key key)
{
    uint16_t key_id = key.getRaw() & 0x00FF; // Tomamos sólo el HID keycode (parte baja)

    // Rango HID de modificadores: 224 (0xE0) a 231 (0xE7)
    return (key_id >= 0xE0 && key_id <= 0xE7);
}

bool ActionsDriver::key_can_interrupt(const Key &Action)
{
    auto ranges_t = static_cast<Utils::KeyRanges>(find_key_type(Action.getRaw()));
    switch (ranges_t)
    {
    case Utils::KeyRanges::LAYER_LOCK:
    case Utils::KeyRanges::DYNAMIC_MACRO:
    case Utils::KeyRanges::LAYER_SHIFT:
    case Utils::KeyRanges::ALPHA_WITH_MODIFIERS:
    {
        // NRF_LOG_DEBUG("Not superkey key  is NOT interruptable ");
        return false;
    }
    break;

    default:
    {
        // NRF_LOG_DEBUG("Not superkey key  is interruptable ");
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

void ActionsDriver::send_regular_key(const Key &key, const KeyAddr &key_addr)
{
    uint8_t modif = (key.getRaw() & 0xFF00) >> 8;

    if (key.getRaw() >= 256 && key.getRaw() <= 7935)
    {
        if (modif & 0x01)
        {
            handleKeyswitchEvent(Key_LeftControl, key_addr, IS_PRESSED | WAS_PRESSED | INJECTED);
        }

        if (modif & 0x02)
        {
            handleKeyswitchEvent(Key_LeftAlt, key_addr, IS_PRESSED | WAS_PRESSED | INJECTED);
        }

        if (modif & 0x04)
        {
            handleKeyswitchEvent(Key_RightAlt, key_addr, IS_PRESSED | WAS_PRESSED | INJECTED);
        }

        if (modif & 0x08)
        {
            handleKeyswitchEvent(Key_LeftShift, key_addr, IS_PRESSED | WAS_PRESSED | INJECTED);
            NRF_LOG_DEBUG("Left Shift pressed");
        }

        if (modif & 0x10)
        {
            handleKeyswitchEvent(Key_LeftGui, key_addr, IS_PRESSED | WAS_PRESSED | INJECTED);
        }
        handleKeyswitchEvent(key, key_addr, IS_PRESSED | WAS_PRESSED | INJECTED);
    }
}

void ActionsDriver::send_modifiers_from_flags(uint8_t modif_flags, const KeyAddr &key_addr)
{
    if (modif_flags & CTRL_HELD)
        handleKeyswitchEvent(Key_LeftControl, key_addr, IS_PRESSED | INJECTED);
    if (modif_flags & LALT_HELD)
        handleKeyswitchEvent(Key_LeftAlt, key_addr, IS_PRESSED | INJECTED);
    if (modif_flags & RALT_HELD)
        handleKeyswitchEvent(Key_RightAlt, key_addr, IS_PRESSED | INJECTED);
    if (modif_flags & SHIFT_HELD)
        handleKeyswitchEvent(Key_LeftShift, key_addr, IS_PRESSED | INJECTED);
    if (modif_flags & GUI_HELD)
        handleKeyswitchEvent(Key_LeftGui, key_addr, IS_PRESSED | INJECTED);

    kaleidoscope::Runtime.hid().keyboard().sendReport();
}

void ActionsDriver::send_modifier(const Key &key, const KeyAddr &key_addr)
{
    /* Si presionamos un modificador y luego una SK, debemos poder liberar esta SK con el modificador asociado */
    if (key.getRaw() < Utils::ALPHA_KEYS)
    {
        uint8_t modif = (key.getRaw() & 0xFF00) >> 8;

        if (modif & 0x01)
            handleKeyswitchEvent(Key_LeftControl, key_addr, IS_PRESSED | INJECTED);
        if (modif & 0x02)
            handleKeyswitchEvent(Key_LeftAlt, key_addr, IS_PRESSED | INJECTED);
        if (modif & 0x04)
            handleKeyswitchEvent(Key_RightAlt, key_addr, IS_PRESSED | INJECTED);
        if (modif & 0x08)
            handleKeyswitchEvent(Key_LeftShift, key_addr, IS_PRESSED | INJECTED);
        if (modif & 0x10)
            handleKeyswitchEvent(Key_LeftGui, key_addr, IS_PRESSED | INJECTED);
        if (modif & 0x20)
            handleKeyswitchEvent(Key_RightControl, key_addr, IS_PRESSED | INJECTED);
        if (modif & 0x40)
            handleKeyswitchEvent(Key_RightShift, key_addr, IS_PRESSED | INJECTED);
        if (modif & 0x80)
            handleKeyswitchEvent(Key_RightGui, key_addr, IS_PRESSED | INJECTED);

        kaleidoscope::Runtime.hid().keyboard().sendReport();
    }
}

void ActionsDriver::send_key_twice(const Key &key, const KeyAddr &key_addr)
{
    handleKeyswitchEvent(key, key_addr, IS_PRESSED | INJECTED);
    kaleidoscope::Runtime.hid().keyboard().sendReport();
    handleKeyswitchEvent(key, key_addr, WAS_PRESSED | INJECTED);
    kaleidoscope::Runtime.hid().keyboard().sendReport();
    handleKeyswitchEvent(key, key_addr, IS_PRESSED | INJECTED);
    kaleidoscope::Runtime.hid().keyboard().sendReport();
    handleKeyswitchEvent(key, key_addr, WAS_PRESSED | INJECTED);
}