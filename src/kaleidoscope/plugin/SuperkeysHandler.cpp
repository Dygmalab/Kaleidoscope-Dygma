//
// Created by Urano on 26/12/2023.
//

#include "SuperkeysHandler.h"

namespace kaleidoscope
{
namespace plugin
{

EventHandlerResult SuperkeysHandler::onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t keyState)
{
    return EventHandlerResult::OK;
}

EventHandlerResult SuperkeysHandler::onFocusEvent(const char *command)
{
    return EventHandlerResult::OK;
}

EventHandlerResult SuperkeysHandler::beforeReportingState()
{
    return EventHandlerResult::OK;
}

void SuperkeysHandler::setup(uint8_t dynamic_offset, uint16_t size)
{
}

void SuperkeysHandler::init()
{
}

void SuperkeysHandler::config()
{
}

void SuperkeysHandler::enable()
{
}

void SuperkeysHandler::disable()
{
}

void SuperkeysHandler::run()
{
}

} // namespace plugin
} // namespace kaleidoscope
kaleidoscope::plugin::SuperkeysHandler SuperkeysHandler;