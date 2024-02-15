//
// Created by Urano on 28/12/2023.
//

#ifndef NRF_NEURON_INCLUDES_H
#define NRF_NEURON_INCLUDES_H
#include <Kaleidoscope.h>
#include <Kaleidoscope-Ranges.h>
namespace Utils{
struct Actions
{
    Key tap;
    Key hold;
    Key tap_hold;
    Key double_tap;
    Key double_tap_hold;
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
    TAP,
    HOLD,
    IDLE
};
}

#include "Kaleidoscope/src/kaleidoscope/plugin/Superkeys/Actions/ActionsDriver.h"
#include "libraries/Kaleidoscope/src/kaleidoscope/plugin/Superkeys/Superkey/Superkey.h"
#endif // NRF_NEURON_INCLUDES_H
