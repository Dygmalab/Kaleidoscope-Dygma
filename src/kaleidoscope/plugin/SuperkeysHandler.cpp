/* SuperkeysHandler - SuperKeys support for Kaleidoscope.
 * Copyright (C) 2025 Dygma Lab S.L.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Superkeys/includes.h"
#include "Superkeys/Superkey/Superkey.h"
#include "Superkeys/Actions/ActionsDriver.h"
#include <cstdint>
// #pragma GCC push_options
// #pragma GCC optimize("O0") // No optimization

#include "kaleidoscope/plugin/SuperkeysHandler.h"
#include "SuperkeysHandler.h"
#include "kaleidoscope/plugin/Qukeys.h"

namespace kaleidoscope
{
namespace plugin
{
SuperkeysHandler::Configurations configurations;
uint16_t SuperkeysHandler::settings_base_ = 0;
uint8_t SuperkeysHandler::configured_superkeys = 0;

Superkey SuperkeysHandler_sk_array[Utils::MAX_SUPER_KEYS_ACTIVE];

// Shared configuration for all Superkeys
static Utils::SharedConfig shared_sk_config;

Key SuperkeysHandler::Actions[6] = {};
uint8_t super_key_index = 0;
uint8_t SuperkeysHandler::cache_modifiers = 0;

void SuperkeysHandler::setup(uint8_t active_superkeys, const Superkey::superkey_config_t * p_sk_map )
{
    configured_superkeys = active_superkeys;
    settings_base_ = kaleidoscope::plugin::EEPROMSettings::requestSlice(sizeof(SuperkeysHandler::Configurations));
    cleanup();
    config();
    init(p_sk_map);
}

void SuperkeysHandler::init(const Superkey::superkey_config_t * p_sk_map)
{

    //NRF_LOG_INFO("SIZE OF Superkey: %i", sizeof(Superkey));
    //NRF_LOG_INFO("SIZE OF Superkey array: %i", sizeof(SuperkeysHandler_sk_array));
    //NRF_LOG_INFO("SIZE OF Superkey map: %i", sizeof(sk_map));

    // Update shared configuration
    shared_sk_config.hold_start_ = configurations.hold_start_;
    shared_sk_config.time_out_ = configurations.time_out_;
    shared_sk_config.overlap_threshold_ = configurations.overlap_threshold_;
    
    uint16_t sk_index = 0;
    uint16_t max_sk = get_configured_sk();
    
    // Limit to array size
    if (max_sk > Utils::MAX_SUPER_KEYS_ACTIVE) {
        //NRF_LOG_WARNING("Configured superkeys (%i) exceeds array size (%i), limiting to %i", max_sk, Utils::MAX_SUPER_KEYS_ACTIVE, Utils::MAX_SUPER_KEYS_ACTIVE);
        max_sk = Utils::MAX_SUPER_KEYS_ACTIVE;
        configured_superkeys = Utils::MAX_SUPER_KEYS_ACTIVE;
    }
    
    while (sk_index < max_sk)
    {
        // Initialize superkey in static array with pointer directly to sk_map
        // No need to copy - sk_map persists for the lifetime of the program
        SuperkeysHandler_sk_array[sk_index] = Superkey(sk_index, &shared_sk_config, &p_sk_map[sk_index]);
        SuperkeysHandler_sk_array[sk_index].init(&p_sk_map[sk_index]);
        
        sk_index++;
    }
    
    //NRF_LOG_INFO("Initialized %i superkeys in static array", sk_index);
    //NRF_LOG_FLUSH();
}

void SuperkeysHandler::config()
{
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

void SuperkeysHandler::save_configurations(const Superkey::superkey_config_t * p_sk_map)
{
    Runtime.storage().put(settings_base_, configurations);
    Runtime.storage().commit();
    config();
    
    // Update shared configuration for all superkeys
    shared_sk_config.hold_start_ = configurations.hold_start_;
    shared_sk_config.time_out_ = configurations.time_out_;
    shared_sk_config.overlap_threshold_ = configurations.overlap_threshold_;
    
    if(p_sk_map != nullptr)
    {
        cleanup();
        init(p_sk_map);
    }
}

void SuperkeysHandler::save_superkey_map_from(const Superkey::superkey_config_t * p_sk_map, uint8_t active_superkeys)
{
    configured_superkeys = active_superkeys;
    if(p_sk_map != nullptr)
    {
        cleanup();
        init(p_sk_map);
    }
}

uint8_t SuperkeysHandler::get_configured_sk()
{
    return configured_superkeys;
}

void SuperkeysHandler::cleanup()
{
    // Reset all superkeys to default state
    for (uint16_t i = 0; i < Utils::MAX_SUPER_KEYS_ACTIVE; ++i)
    {
        SuperkeysHandler_sk_array[i] = Superkey();
    }
}

void SuperkeysHandler::save_pressed_modifiers(Key &mapped_key, uint8_t keyState)
{
    uint16_t raw = mapped_key.getRaw() & 0x00FF; // Take only the HID keycode (lower part)

    // If it is a modifier, we update cache_modifiers according to the current state
    if (keyState == KEY_PRESED)
    {
        // Add corresponding flag
        switch (raw)
        {
            case HID_KEYBOARD_LEFT_CONTROL:
                cache_modifiers |= CTRL_HELD;
                break;
            case HID_KEYBOARD_LEFT_ALT:
                cache_modifiers |= LALT_HELD;
                break;
            case HID_KEYBOARD_RIGHT_ALT:
                cache_modifiers |= RALT_HELD;
                break;
            case HID_KEYBOARD_LEFT_SHIFT:
                cache_modifiers |= SHIFT_HELD;
                break;
            case HID_KEYBOARD_LEFT_GUI:
                cache_modifiers |= GUI_HELD;
                break;
            case HID_KEYBOARD_RIGHT_CONTROL:
                cache_modifiers |= CTRL_HELD;
                break;
            case HID_KEYBOARD_RIGHT_SHIFT:
                cache_modifiers |= SHIFT_HELD;
                break;
            case HID_KEYBOARD_RIGHT_GUI:
                cache_modifiers |= GUI_HELD;
                break;
        }
    }
    else if (keyState == KEY_RELEASED)
    {
        // Remove flag if not pressed
        switch (raw)
        {
            case HID_KEYBOARD_LEFT_CONTROL:
                cache_modifiers &= ~CTRL_HELD;
                break;
            case HID_KEYBOARD_LEFT_ALT:
                cache_modifiers &= ~LALT_HELD;
                break;
            case HID_KEYBOARD_RIGHT_ALT:
                cache_modifiers &= ~RALT_HELD;
                break;
            case HID_KEYBOARD_LEFT_SHIFT:
                cache_modifiers &= ~SHIFT_HELD;
                break;
            case HID_KEYBOARD_LEFT_GUI:
                cache_modifiers &= ~GUI_HELD;
                break;
            case HID_KEYBOARD_RIGHT_CONTROL:
                cache_modifiers &= ~CTRL_HELD;
                break;
            case HID_KEYBOARD_RIGHT_SHIFT:
                cache_modifiers &= ~SHIFT_HELD;
                break;
            case HID_KEYBOARD_RIGHT_GUI:
                cache_modifiers &= ~GUI_HELD;
                break;
        }
    }

}

void SuperkeysHandler::set_minimum_hold(uint16_t minimum_hold)
{
    configurations.overlap_threshold_ = minimum_hold;
    save_configurations(nullptr);
}

EventHandlerResult SuperkeysHandler::handle_superkeys(Key &mapped_key, KeyAddr key_addr, uint8_t keyState)
{
    // Superkey processing starts here.
    super_key_index = static_cast<uint8_t>(mapped_key.getRaw() - ranges::DYNAMIC_SUPER_FIRST);

    if (keyToggledOn(keyState))
    {
        //NRF_LOG_DEBUG("super_key_index %i  ", super_key_index);
        for (uint8_t pos = 0; pos < get_configured_sk(); ++pos)
        {
            if (SuperkeysHandler_sk_array[pos].get_index() == super_key_index)
            {
                if (!SuperkeysHandler_sk_array[pos].is_enable())
                {
                    // Normal arm path
                    SuperkeysHandler_sk_array[pos].enable(cache_modifiers);
                    SuperkeysHandler_sk_array[pos].init_timer();
                    SuperkeysHandler_sk_array[pos].set_key_and_keyAddr(mapped_key, key_addr);
                    SuperkeysHandler_sk_array[pos].key_pressed();

                    Utils::TimelineEntry entry = {
                        mapped_key, key_addr, Runtime.millisAtCycleStart(), Utils::KeyType::SUPERKEY, false, static_cast<void *>(&SuperkeysHandler_sk_array[pos])};

                    timeline.add(entry);
                    return EventHandlerResult::EVENT_CONSUMED;
                }
                else
                {
                    // Already enabled
                    if (SuperkeysHandler_sk_array[pos].is_qukey())
                    {
                        // For qukeys, re-arm to avoid stale enabled state after a hold
                        // causing the first next press to be ignored.
                        SuperkeysHandler_sk_array[pos].disable();

                        SuperkeysHandler_sk_array[pos].enable(cache_modifiers);
                        SuperkeysHandler_sk_array[pos].init_timer();
                        SuperkeysHandler_sk_array[pos].set_key_and_keyAddr(mapped_key, key_addr);
                        SuperkeysHandler_sk_array[pos].key_pressed();

                        Utils::TimelineEntry entry = {
                            mapped_key, key_addr, Runtime.millisAtCycleStart(), Utils::KeyType::SUPERKEY, false, static_cast<void *>(&SuperkeysHandler_sk_array[pos])};

                        timeline.add(entry);
                        return EventHandlerResult::EVENT_CONSUMED;
                    }
                    else
                    {
                        // Non-qukeys: check if it's in a finalized state (holded/triggered)
                        // If so, re-arm it like qukeys to allow a new press after hold
                        if (SuperkeysHandler_sk_array[pos].is_holded() || SuperkeysHandler_sk_array[pos].is_triggered())
                        {
                            SuperkeysHandler_sk_array[pos].disable();

                            SuperkeysHandler_sk_array[pos].enable(cache_modifiers);
                            SuperkeysHandler_sk_array[pos].init_timer();
                            SuperkeysHandler_sk_array[pos].set_key_and_keyAddr(mapped_key, key_addr);
                            SuperkeysHandler_sk_array[pos].key_pressed();

                            Utils::TimelineEntry entry = {
                                mapped_key, key_addr, Runtime.millisAtCycleStart(), Utils::KeyType::SUPERKEY, false, static_cast<void *>(&SuperkeysHandler_sk_array[pos])};

                            timeline.add(entry);
                            return EventHandlerResult::EVENT_CONSUMED;
                        }
                        else
                        {
                            // Normal case: forward press to accumulate tap_count
                            SuperkeysHandler_sk_array[pos].key_pressed();
                            return EventHandlerResult::EVENT_CONSUMED;
                        }
                    }
                }
            }
        }
    }
    else if (keyToggledOff(keyState))
    {
        for (uint8_t pos = 0; pos < get_configured_sk(); ++pos)
        {
            if (SuperkeysHandler_sk_array[pos].get_index() == super_key_index)
            {
                SuperkeysHandler_sk_array[pos].key_released();
                return EventHandlerResult::EVENT_CONSUMED;
            }
        }
    }
    else if (keyIsPressed(keyState))
    {
        for (uint8_t pos = 0; pos < get_configured_sk(); ++pos)
        {
            if (SuperkeysHandler_sk_array[pos].get_index() == super_key_index)
            {
                SuperkeysHandler_sk_array[pos].key_is_pressed();
                return EventHandlerResult::EVENT_CONSUMED;
            }
        }
    }
    return EventHandlerResult::OK;
}

EventHandlerResult SuperkeysHandler::handle_regular_keys(Key &mapped_key, KeyAddr key_addr, uint8_t keyState)
{

    if (keyToggledOn(keyState))
    {
        if (ActionsDriver::isOnlyModifier(mapped_key))
        {
            save_pressed_modifiers(mapped_key, keyState);
        }

        uint16_t raw = mapped_key.getRaw();
        if (raw >= Utils::LED_BUTTONS_FIRST && raw <= Utils::LED_BUTTONS_LAST)
        {
            return EventHandlerResult::OK;
        }

        // Don't add Dynamic Macros to timeline - they have their own event handler
        // that consumes them. Adding them to timeline causes double execution.
        if (mapped_key.getRaw() >= ranges::DYNAMIC_MACRO_FIRST && 
            mapped_key.getRaw() <= ranges::DYNAMIC_MACRO_LAST)
        {
            return EventHandlerResult::OK;
        }

        Utils::TimelineEntry entry = {mapped_key, key_addr, Runtime.millisAtCycleStart(), Utils::KeyType::NORMAL, false, nullptr};

        if (timeline.add(entry))
        {
            return EventHandlerResult::EVENT_CONSUMED;
        }
    }
    else if (keyToggledOff(keyState))
    {
        if (ActionsDriver::isOnlyModifier(mapped_key))
        {
            save_pressed_modifiers(mapped_key, keyState);
        }
        // If the key is toggled off, we remove it from the timeline.
        timeline.remove(key_addr);
    }
    else if (keyIsPressed(keyState))
    {
        // If the key is pressed, we do nothing.
    }
    return EventHandlerResult::OK;
}

EventHandlerResult SuperkeysHandler::onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t keyState)
{
    EventHandlerResult result = EventHandlerResult::OK;
    // If k is not a physical key, ignore it; some other plugin injected it.
    if (keyState & INJECTED)
    {
        return EventHandlerResult::OK;
    }

    // If it's not a super-key press, we treat it here.
    if (IS_OUTSIDE_DYNAMIC_SUPER_RANGE(mapped_key.getRaw()))
    {
        result = handle_regular_keys(mapped_key, key_addr, keyState);
        return result;
    }

    result = handle_superkeys(mapped_key, key_addr, keyState);

    return result;
}

EventHandlerResult SuperkeysHandler::beforeReportingState()
{
    // Process superkeys in timeline order to preserve correct finalization sequence
    // This ensures that earlier superkeys finalize before later ones check for pending superkeys
    timeline.process_superkeys_in_order();

    return EventHandlerResult::OK;
}

EventHandlerResult SuperkeysHandler::onFocusEvent(const char *command)
{
    if (::Focus.handleHelp(command, "superkeys.map\nsuperkeys.waitfor\nsuperkeys.timeout\nsuperkeys.repeat\nsuperkeys.holdstart\nsuperkeys.overlap"))
        return EventHandlerResult::OK;

    if (strncmp_P(command, "superkeys.", 10) != 0) return EventHandlerResult::OK;


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
                save_configurations(nullptr);
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
            save_configurations(nullptr);
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
            save_configurations(nullptr);
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
            save_configurations(nullptr);
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
            uint16_t overlap_threshold = 0;
            ::Focus.read(overlap_threshold);
            configurations.overlap_threshold_ = overlap_threshold;
            save_configurations(nullptr);
        }
    }

    return EventHandlerResult::EVENT_CONSUMED;
}

} // namespace plugin
} // namespace kaleidoscope
kaleidoscope::plugin::SuperkeysHandler superkeysHandler;
