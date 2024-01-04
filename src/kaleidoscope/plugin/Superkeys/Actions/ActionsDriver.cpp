//
// Created by Urano on 26/12/2023.
//

#include "ActionsDriver.h"

void ActionsDriver::event_handler(ActionsDriver::EventType event_t, Superkey*  superkey)
{
    switch (event_t)
    {
        case ActionsDriver::EventType::IDLE:
        {
            break;
        }
        case ActionsDriver::EventType::TAP:
        {
            superkey->set_tap_action();
            break;
        }
        case ActionsDriver::EventType::RELEASE:
        {
            superkey->set_tap_action();
            break;
        }
        case ActionsDriver::EventType::HOLD:
        {
            break;
        }
        case ActionsDriver::EventType::TIMEOUT:
        {
            break;
        }
    }
}
