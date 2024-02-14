//
// Created by Urano on 26/12/2023.
//

#include "ActionsDriver.h"

void ActionsDriver::return_type(uint8_t tap_count, EventType action)
{
    DynamicSuperKeys::SuperType result;
    if (action == EventType::TAP)
    {
        switch (previous)
        {
            case DynamicSuperKeys::None:
                result = DynamicSuperKeys::Tap_Once;
                break;
            case DynamicSuperKeys::Tap_Once:
                result = DynamicSuperKeys::Tap_Twice;
                break;
            case DynamicSuperKeys::Tap_Twice:
                result = DynamicSuperKeys::Tap_Trice;
                break;
            default:
                result = DynamicSuperKeys::Tap_Trice;
        }
    }
    if (action == EventType::HOLD)
    {
        switch (previous)
        {
            case DynamicSuperKeys::None:
                result = DynamicSuperKeys::None;
                break;
            case DynamicSuperKeys::Tap_Once:
                result = DynamicSuperKeys::Hold_Once;
                break;
            case DynamicSuperKeys::Tap_Twice:
                result = DynamicSuperKeys::Tap_Hold;
                break;
            case DynamicSuperKeys::Tap_Trice:
                result = DynamicSuperKeys::Tap_Twice_Hold;
                break;
            default:
                result = DynamicSuperKeys::Tap_Twice_Hold;
        }
    }
    return result;
}
