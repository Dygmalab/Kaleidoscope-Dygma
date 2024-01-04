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
        TIMEOUT
    };

    enum class Actions
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
    static void event_handler(EventType event_t, Superkey*  superkey);
};


#endif // NRF_NEURON_ACTIONSDRIVER_H
