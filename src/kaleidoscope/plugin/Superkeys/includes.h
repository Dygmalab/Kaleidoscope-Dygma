#ifndef NRF_NEURON_INCLUDES_H
#define NRF_NEURON_INCLUDES_H
#include <Kaleidoscope.h>
#include <Kaleidoscope-Ranges.h>

#define KEY_PRESED 2
#define KEY_RELEASED 1

namespace Utils
{
    struct Actions
    {
        Key tap;
        Key hold;
        Key tap_hold;
        Key double_tap;
        Key double_tap_hold;
    };

    struct ExtendedActions
    {
        Key key;
        bool key_is_interruptable;
        bool release_two_keys; // This is used if the sk has no key set it in double tap so we will release the tap action twice.
    };

    enum class TapType
    {
        None,
        Hold_Once,
        Tap_Once,
        Tap_Hold,
        Tap_Twice,
        Tap_Twice_Hold,
        Tap_Trice,
    };

    enum class EventType
    {
        IDLE = 0,
        HOLD = 1,
        TAP = 2,
    };

    enum : uint16_t
    {
        ALPHA_KEYS = 255,
        ALPHA_WITH_MODIFIERS_FIRST = 256,
        ALPHA_WITH_MODIFIERS_LAST = 7935,
        LED_BUTTONS_FIRST = 17152,
        PREVIOUS_LED_EFFECT,
        LED_BUTTONS_LAST,
        LAYER_SHIFT_FIRST = 17450,
        LAYER_SHIFT_LAST = 17459,
        LAYER_LOCK_FIRST = 17492,
        LAYER_LOCK_LAST = 17501
    };

    struct SKRanges
    {
        uint16_t start;
        uint16_t end;
    };

    static constexpr SKRanges ranges[] = {
        {LAYER_LOCK_FIRST, LAYER_LOCK_LAST},
        {LED_BUTTONS_FIRST, LED_BUTTONS_LAST},
        {kaleidoscope::ranges::DYNAMIC_MACRO_FIRST, kaleidoscope::ranges::DYNAMIC_MACRO_LAST},
        {ALPHA_WITH_MODIFIERS_FIRST, ALPHA_WITH_MODIFIERS_LAST},
        {0, ALPHA_KEYS},
        {LAYER_SHIFT_FIRST, LAYER_SHIFT_LAST},
        {23785, 23786} // maybe these are LEDs buttons.
    };

    static constexpr int numRanges = sizeof(ranges) / sizeof(ranges[0]);

    enum class KeyRanges
    {
        LAYER_LOCK,
        LED_BUTTONS,
        DYNAMIC_MACRO,
        ALPHA_WITH_MODIFIERS,
        ALPHA_KEYS,
        LAYER_SHIFT,
        UNKNOW
    };

    enum class KeyType {
        NONE,
        NORMAL,
        SUPERKEY,
        MACRO,
        SPECIAL,
        MODIFIER
    };

    struct TimelineEntry
    {
        Key key;
        KeyAddr addr;
        uint32_t timestamp;
        KeyType type;
        bool is_interruptible; // Solo relevante para Superkeys
        void* context; // Apunta a la instancia que gestiona esta key (Superkey*, Macro*, etc.)
    };
}
#include "Timeline/Timeline.h"
#include "Actions/ActionsDriver.h"
#include "Superkey/Superkey.h"
#endif // NRF_NEURON_INCLUDES_H
