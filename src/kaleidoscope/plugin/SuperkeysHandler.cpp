//
// Created by Urano on 26/12/2023.
//

#include "SuperkeysHandler.h"

#define IS_OUTSIDE_DYNAMIC_SUPER_RANGE(key) ((key) < ranges::DYNAMIC_SUPER_FIRST || (key) > ranges::DYNAMIC_SUPER_LAST)

namespace kaleidoscope
{
namespace plugin
{
SuperkeysHandler::Configurations configurations;
uint16_t SuperkeysHandler::settings_base_ = 0;
uint8_t SuperkeysHandler::active_superkeys = 0;
Superkey *SuperkeysHandler::Sk_queue[SuperkeysHandler::MAX_SUPER_KEYS_ACTIVE] = {};
Key SuperkeysHandler::Actions[6] = {};
uint8_t super_key_index = 0;

void SuperkeysHandler::setup()
{
    cleanup();
    config();
    init();
}

void SuperkeysHandler::init()
{
    set_active_sk();
    uint16_t pos = 0;
    NRF_LOG_DEBUG("Configured Super-keys %i", get_active_sk());
    while (pos < get_active_sk())
    {
        // Set Superkeys keys.
        for (int i = 0; i < KEYS_IN_SUPERKEY; ++i)
        {
            Actions[i] = configurations.keys[pos][i];
        }
        // Create a new superkey instance giving the position as index, we need to use a C style array and new due to the compatibility with Raise 1.
        Superkey *superkeyInstance = new Superkey(pos, configurations.hold_start_, configurations.time_out_);
        superkeyInstance->init(Actions);

        // Add instance to the queue
        if (Sk_queue[pos] == nullptr)
        {
            Sk_queue[pos] = superkeyInstance;
        }
        else
        {
            NRF_LOG_DEBUG("ERROR");
        }
        pos++;
    }
}

void SuperkeysHandler::config()
{
    settings_base_ = kaleidoscope::plugin::EEPROMSettings::requestSlice(sizeof(configurations));
    Runtime.storage().get(settings_base_, configurations);

    // if one block is invalid, restart everything
    if (configurations.hold_start_ == 0xFFFF)
    {

        configurations.reset();
        Runtime.storage().put(settings_base_, configurations);
        Runtime.storage().commit();
    }
    Runtime.storage().get(settings_base_, configurations);
}

void SuperkeysHandler::enable()
{
    // enable sk so it can be treated.
}

void SuperkeysHandler::disable()
{
    // disable sk so it can't be treated.
}

void SuperkeysHandler::save_configurations()
{
    Runtime.storage().put(settings_base_, configurations);
    Runtime.storage().commit();
    setup();
}

void SuperkeysHandler::set_active_sk()
{
    active_superkeys = 0;
    uint8_t undefined_actions = 0;
    for (uint16_t i = 0; i < SUPER_KEY_COUNT; ++i)
    {
        for (int j = 0; j < KEYS_IN_SUPERKEY; ++j)
        {
            if (configurations.keys[i][j] == 0xFFFF)
            {
                undefined_actions++;
            }
            if (undefined_actions == 5)
            {
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

void SuperkeysHandler::cleanup()
{
    for (uint16_t i = 0; i < get_active_sk(); ++i)
    {
        delete Sk_queue[i];
        Sk_queue[i] = nullptr; // Assign nullptr after deletion to avoid dangling pointer issues.
    }
}

EventHandlerResult SuperkeysHandler::onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t keyState)
{
    // If k is not a physical key, ignore it; some other plugin injected it.
    if (keyState & INJECTED)
    {
        return EventHandlerResult::OK;
    }

    // If it's not a super-key press , we treat it here.
    if (IS_OUTSIDE_DYNAMIC_SUPER_RANGE(mapped_key.getRaw()))
    {
        // TODO: treat normal, keys.
        return EventHandlerResult::OK;
    }

    super_key_index = static_cast<uint8_t>(mapped_key.getRaw() - ranges::DYNAMIC_SUPER_FIRST);

    if (keyToggledOn(keyState))
    {
        NRF_LOG_DEBUG("super_key_index %i  ", super_key_index);
        for (uint8_t pos = 0; pos <= get_active_sk(); ++pos)
        {

            if (Sk_queue[pos]->get_index() == super_key_index && !Sk_queue[pos]->is_enable())
            {
                // We want to enable the superkey one time,
                // so if the superkey wasn't enabled,
                // we enable it, otherwise continue.
                Sk_queue[pos]->enable();
                Sk_queue[pos]->init_timer();
                Sk_queue[pos]->key_pressed();
                return EventHandlerResult::EVENT_CONSUMED;
            }
        }
    }
    else if (keyToggledOff(keyState))
    {
        for (uint8_t pos = 0; pos <= get_active_sk(); ++pos)
        {

            if (Sk_queue[pos]->get_index() == super_key_index && !Sk_queue[pos]->is_enable())
            { // We want to enable the superkey one time, so if the superkey wasn't enabled, we enable it, otherwise continue.
                Sk_queue[pos]->key_released();
                return EventHandlerResult::EVENT_CONSUMED;
            }
        }
    }
    else if (keyIsPressed(keyState))
    {
        for (uint8_t pos = 0; pos <= get_active_sk(); ++pos)
        {

            if (Sk_queue[pos]->get_index() == super_key_index)
            { // We want to enable the superkey one time, so if the superkey wasn't enabled, we enable it, otherwise continue.
                Sk_queue[pos]->key_is_pressed();
                return EventHandlerResult::EVENT_CONSUMED;
            }
        }
    }
    return EventHandlerResult::OK;
}

EventHandlerResult SuperkeysHandler::beforeReportingState()
{
    static uint8_t pos = 0;
    // Iterate throw every superkey if they are enabled.
    if (Sk_queue[pos]->is_enable())
    {
        Sk_queue[pos]->run();
    }
    ++pos;
    if (pos >= get_active_sk())
    {
        pos = 0;
    }
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
                for (int j = 0; j < KEYS_IN_SUPERKEY; ++j)
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
                    pos = (pos / 6) * 6; // Reinicia a la primera columna de la siguiente fila
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
                configurations.wait_for_ = wait;
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


} // namespace plugin
} // namespace kaleidoscope
kaleidoscope::plugin::SuperkeysHandler SuperkeysHandler;