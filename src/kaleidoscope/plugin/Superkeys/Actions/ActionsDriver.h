//
// Created by Urano on 26/12/2023.
//

#ifndef NRF_NEURON_ACTIONSDRIVER_H
#define NRF_NEURON_ACTIONSDRIVER_H

#include "kaleidoscope/src/kaleidoscope/plugin/Superkeys/Superkey/Superkey.h"
#include <cstdint>

using EventFunction = uint8_t (*)();

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

    static void event_handler(const Superkey &superkey, EventType event_t, EventFunction event)
    {
        switch (event_t)
        {
            case EventType::IDLE:
            {
                break;
            }
            case EventType::TAP:
            case EventType::RELEASE:
            {
                uint8_t tap_count = event();
                calculate_action(superkey, tap_count);
                break;
            }
            case EventType::HOLD:
            {
                break;
            }
            case EventType::TIMEOUT:
            {
                break;
            }
        }
    }

  private:
    // Variables
    struct
    {
        bool tap = false;
        bool hold = false;
        bool tap_hold = false;
        bool double_tap = false;
        bool double_tap_hold = false;
    };

    static void calculate_action(const Superkey &superkey, uint8_t count)
    {
        // call the corresponding action in SK class.
        superkey.tap_action();
    }

};


#endif // NRF_NEURON_ACTIONSDRIVER_H
