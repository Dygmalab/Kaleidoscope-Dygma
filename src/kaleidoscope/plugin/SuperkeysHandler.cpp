//
// Created by Urano on 26/12/2023.
//

#include "SuperkeysHandler.h"


namespace kaleidoscope
{
namespace plugin
{
SuperkeysHandler::Configurations configurations;
uint16_t SuperkeysHandler::settings_base_ = 0;
uint8_t SuperkeysHandler::active_superkeys = 0;

void SuperkeysHandler::setup()
{
    config();
    init();
}

EventHandlerResult SuperkeysHandler::onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t keyState)
{
    if (keyToggledOn(keyState))
    {
        NRF_LOG_DEBUG("Configured Super-keys %i", get_active_sk());d
    }
    else if (keyToggledOff(keyState))
    {
    }
    else if (keyIsPressed(keyState))
    {
    }
    return EventHandlerResult::OK;
}

EventHandlerResult SuperkeysHandler::beforeReportingState()
{
    //Iterate throw every superkey.
    return EventHandlerResult::OK;
}

EventHandlerResult SuperkeysHandler::onFocusEvent(const char *command)
{
    if (::Focus.handleHelp(command, "superkeys.map\ntuperkeys.waitfor\ntsuperkeys.timeout\ntsuperkeys.repeat\ntsuperkeys.holdstart\ntsuperkeys.overlap"))
        return EventHandlerResult::OK;

    if (strncmp_P(command, "superkeys.", 10) != 0) return EventHandlerResult::OK;

    if (strcmp_P(command + 10, "map") == 0)
    {
        if (::Focus.isEOL())
        {
            Kaleidoscope.storage().get(settings_base_, configurations);
            for (uint16_t i = 0; i < SUPER_KEY_COUNT; ++i)
            {
                for (int j = 0; j < KEY_PER_ACTION; ++j)
                {
                    ::Focus.send(configurations.keys[i][j]);
                }
            }
        }
        else
        {
            uint16_t pos = 0;
            Key key;

            while (!::Focus.isEOL())
            {
                ::Focus.read(key);
                configurations.keys[pos / 6][pos % 6] = key;
                pos++;
                if (pos % 6 == 0)
                {
                    pos = (pos / 6) * 6;  // Reinicia a la primera columna de la siguiente fila
                }
            }
            save_configurations();
        }
    }
    if (strcmp_P(command + 10, "waitfor") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(configurations.wait_for_);
        }
        else
        {
            uint16_t wait = 0;
            ::Focus.read(wait);
            if (configurations.wait_for_ < 2000)
            {
                configurations.wait_for_  = wait;
                save_configurations();
            }
        }
    }
    if (strcmp_P(command + 10, "timeout") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(configurations.time_out_);
        }
        else
        {
            uint16_t time = 0;
            ::Focus.read(time);
            configurations.time_out_ = time;
            save_configurations();
        }
    }
    if (strcmp_P(command + 10, "holdstart") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(configurations.hold_start_);
        }
        else
        {
            uint16_t hold = 0;
            ::Focus.read(hold);
            configurations.hold_start_ = hold;
            save_configurations();
        }
    }
    if (strcmp_P(command + 10, "repeat") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(configurations.repeat_interval_);
        }
        else
        {
            uint8_t repeat = 0;
            ::Focus.read(repeat);
            configurations.repeat_interval_ = repeat;
            save_configurations();
        }
    }
    if (strcmp_P(command + 10, "overlap") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(configurations.overlap_threshold_);
        }
        else
        {
            uint8_t overlap = 0;
            ::Focus.read(overlap);
            if (overlap <= 0) overlap = 1;
            configurations.overlap_threshold_ = overlap;
            save_configurations();
        }
    }

    return EventHandlerResult::EVENT_CONSUMED;
}

void SuperkeysHandler::init()
{
    set_active_sk();
    uint16_t pos = 0;
    NRF_LOG_DEBUG("Configured Super-keys %i", get_active_sk());
/*    while (pos < MAX_SUPER_KEYS_ACTIVE)
    {
        //Create a new instance of Superkey if we read zero call init methods.

    }*/
}

void SuperkeysHandler::config()
{
    settings_base_ = kaleidoscope::plugin::EEPROMSettings::requestSlice(sizeof(configurations) );
    Runtime.storage().get(settings_base_, configurations);

    //if one block is invalid, restart everything
    if (configurations.hold_start_ == 0xFFFF){

        configurations.reset();
        Runtime.storage().put(settings_base_, configurations);
        Runtime.storage().commit();
    }
    NRF_LOG_DEBUG("Setting superkeys configurations: hold_start %i , time_out_ %i ",configurations.hold_start_,configurations.time_out_);
    Runtime.storage().get(settings_base_, configurations);
}

void SuperkeysHandler::enable()
{
}

void SuperkeysHandler::disable()
{
}

void SuperkeysHandler::save_configurations()
{
    Runtime.storage().put(settings_base_, configurations);
    Runtime.storage().commit();
    init();
}
void SuperkeysHandler::set_active_sk()
{
    active_superkeys = 0;
    uint8_t undefined_actions = 0;
    for (uint16_t i = 0; i < SUPER_KEY_COUNT; ++i)
    {
        for (int j = 0; j < KEY_PER_ACTION; ++j)
        {
            if (configurations.keys[i][j] == 0xFFFF){
                undefined_actions++;
            }
            if (undefined_actions == 5){
              return;
            }
        }
        active_superkeys++;
    }
}

uint8_t SuperkeysHandler::get_active_sk()
{
    return SuperkeysHandler::active_superkeys;
}

} // namespace plugin
} // namespace kaleidoscope
kaleidoscope::plugin::SuperkeysHandler SuperkeysHandler;