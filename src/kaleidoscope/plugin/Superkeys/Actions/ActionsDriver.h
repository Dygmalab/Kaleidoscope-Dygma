#ifndef NRF_NEURON_ACTIONSDRIVER_H
#define NRF_NEURON_ACTIONSDRIVER_H

#include "libraries/Kaleidoscope/src/kaleidoscope/plugin/Superkeys/includes.h"
#include <cstdint>

using EventFunction = uint8_t (*)();
class Superkey;
class ActionsDriver
{
  public:
    // Variables
    enum class EventType
    {
        IDLE,
        TAP,
        RELEASE,
        HOLD,
    };

    enum class TapType
    {
        None,
        Tap_Once,
        Hold_Once,
        Tap_Hold,
        Tap_Twice,
        Tap_Twice_Hold,
        Tap_Trice,
    };

    // Functions
    static void return_type(uint8_t tap_count, EventType action);
};


#endif // NRF_NEURON_ACTIONSDRIVER_H
