#ifndef NRF_NEURON_ACTIONSDRIVER_H
#define NRF_NEURON_ACTIONSDRIVER_H
#include "libraries/Kaleidoscope/src/kaleidoscope/plugin/Superkeys/includes.h"
#include <cstdint>

using EventFunction = uint8_t (*)();
class Superkey;
class ActionsDriver
{
public:
    static Key return_type(uint8_t tap_count, const Key *actions, bool repeat_on_hold);

    static bool action_handler(uint8_t tap_count, const Key *actions, const Key &key, const KeyAddr &keyAddr, bool repeat_on_hold);

    static bool isOnlyModifier(Key key);

    static bool check_if_key_is_repeateable(const Key &key);

    static bool check_if_sk_interruptable(const Key &Action);

    static uint16_t find_key_type(uint16_t value);
};

#endif // NRF_NEURON_ACTIONSDRIVER_H
