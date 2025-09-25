#pragma GCC push_options
#pragma GCC optimize("O0") // No optimization

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
Superkey *SuperkeysHandler::Sk_queue[SuperkeysHandler::MAX_SUPER_KEYS_ACTIVE] = {};
Key SuperkeysHandler::Actions[6] = {};
uint8_t super_key_index = 0;
uint8_t SuperkeysHandler::cache_modifiers = 0;

void SuperkeysHandler::setup()
{
    settings_base_ = kaleidoscope::plugin::EEPROMSettings::requestSlice(sizeof(SuperkeysHandler::Configurations));
    cleanup();
    config();
    init();
}

void SuperkeysHandler::init()
{
    set_active_sk();
    uint16_t sk_index = 0;
    // NRF_LOG_DEBUG("Configured Super-keys %i", get_configured_sk());
    while (sk_index < get_configured_sk())
    {
        // Set Super-keys keys.
        for (int i = 0; i < KEYS_IN_SUPERKEY; ++i)
        {
            Actions[i] = configurations.keys[sk_index][i];
        }
        // Create a new superkey instance giving the position as index, we need to use a C style array and new due to the compatibility with Raise 1.
        Superkey *superkeyInstance = new Superkey(sk_index, configurations.hold_start_, configurations.time_out_, configurations.overlap_threshold_);
        superkeyInstance->init(Actions);

        // Add instance to the queue
        if (Sk_queue[sk_index] == nullptr)
        {
            Sk_queue[sk_index] = superkeyInstance;
        }
        sk_index++;
    }
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

void SuperkeysHandler::save_configurations()
{
    Runtime.storage().put(settings_base_, configurations);
    Runtime.storage().commit();
    cleanup();
    config();
    init();
}

void SuperkeysHandler::send_sk_map()
{
    Kaleidoscope.storage().get(settings_base_, configurations);
    for (uint16_t i = 0; i < Utils::SUPER_KEY_COUNT; ++i)
    {
        for (int j = 0; j < KEYS_IN_SUPERKEY; ++j)
        {
            ::Focus.send(configurations.keys[i][j]);
        }
    }
}

void SuperkeysHandler::save_superkey_map_from(const Key (*src)[KEYS_IN_SUPERKEY], uint16_t src_count)
{
    // 1) calcular filas a copiar
    const uint16_t rows = (src_count < Utils::SUPER_KEY_COUNT) ? src_count : Utils::SUPER_KEY_COUNT;

    // 2) preparar IDLE destino (según tu reset() aquí es 0xFFFF)
    Key idle_dst;
    idle_dst.setRaw(1);

    // 3) limpiar todo el destino
    for (uint16_t i = 0; i < Utils::SUPER_KEY_COUNT; ++i)
    {
        for (uint8_t j = 0; j < KEYS_IN_SUPERKEY; ++j)
        {
            configurations.keys[i][j] = idle_dst;
        }
    }

    // 4) copiar con mapeo de IDLE (1 -> 0xFFFF)
    NRF_LOG_DEBUG("Keys in superkeys (rows=%u)", (unsigned)rows);
    for (uint16_t i = 0; i < rows; ++i)
    {
        for (uint8_t j = 0; j < KEYS_IN_SUPERKEY; ++j)
        {
            Key k = src[i][j];
            if (k.getRaw() == 1)
            { // IDLE de KeyRoleManager
                configurations.keys[i][j] = idle_dst;
            }
            else
            {
                configurations.keys[i][j] = k;
            }
            NRF_LOG_DEBUG("%u ,", (unsigned)configurations.keys[i][j].getRaw());
        }
        NRF_LOG_DEBUG("\n");
        NRF_LOG_FLUSH();
    }

    save_configurations();
}

void SuperkeysHandler::save_superkey_map()
{
    static uint16_t pos = 0;

    while (!::Focus.isEOL())
    {
        Key key;
        ::Focus.read(key);
        configurations.keys[pos / 6][pos % 6] = key;
        pos++;
        if (pos % 6 == 0)
        {
            pos = (pos / 6) * 6; // Reset pos to the next superkey
        }
    }
    save_configurations();
}

void SuperkeysHandler::set_active_sk()
{
    configured_superkeys = 0;
    uint8_t undefined_actions = 0;
    for (uint16_t i = 0; i < Utils::SUPER_KEY_COUNT; ++i) // Iterate through all superkeys
    {
        for (int j = 0; j < KEYS_IN_SUPERKEY; ++j) // Iterate through all keys in the superkey
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
        configured_superkeys++;
    }
}

uint8_t SuperkeysHandler::get_configured_sk()
{
    return SuperkeysHandler::configured_superkeys;
}

void SuperkeysHandler::cleanup()
{
    for (uint16_t i = 0; i < get_configured_sk(); ++i)
    {
        delete Sk_queue[i];
        Sk_queue[i] = nullptr; // Assign nullptr after deletion to avoid dangling pointer issues.
    }
}

void SuperkeysHandler::log_cache_modifiers()
{
    NRF_LOG_DEBUG("Estado de cache_modifiers: 0x%02X", cache_modifiers);

    if (cache_modifiers & CTRL_HELD) NRF_LOG_DEBUG("CTRL activo");
    if (cache_modifiers & LALT_HELD) NRF_LOG_DEBUG("Left ALT activo");
    if (cache_modifiers & RALT_HELD) NRF_LOG_DEBUG("Right ALT activo");
    if (cache_modifiers & SHIFT_HELD) NRF_LOG_DEBUG("SHIFT activo");
    if (cache_modifiers & GUI_HELD) NRF_LOG_DEBUG("GUI activo");
    if (cache_modifiers == 0) NRF_LOG_DEBUG("Sin modificadores");
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
    // Log after updating
    // log_cache_modifiers();
}

void SuperkeysHandler::set_minimum_hold(uint16_t minimum_hold)
{
    configurations.overlap_threshold_ = minimum_hold;
    save_configurations();
}

EventHandlerResult SuperkeysHandler::handle_superkeys(kaleidoscope::Key &mapped_key, KeyAddr key_addr, uint8_t keyState)
{
    // Superkey processing starts here.
    super_key_index = static_cast<uint8_t>(mapped_key.getRaw() - ranges::DYNAMIC_SUPER_FIRST);

    if (keyToggledOn(keyState))
    {
        NRF_LOG_DEBUG("super_key_index %i  ", super_key_index);
        for (uint8_t pos = 0; pos <= get_configured_sk(); ++pos)
        {
            if (Sk_queue[pos]->get_index() == super_key_index && !Sk_queue[pos]->is_enable())
            {
                // We want to enable the superkey one time,
                // so if the superkey wasn't enabled,
                // we enable it, otherwise continue.
                Sk_queue[pos]->enable(cache_modifiers);
                Sk_queue[pos]->init_timer();
                Sk_queue[pos]->set_key_and_keyAddr(mapped_key, key_addr);
                Sk_queue[pos]->key_pressed();

                Utils::TimelineEntry entry = {
                    mapped_key, key_addr, Runtime.millisAtCycleStart(), Utils::KeyType::SUPERKEY, false, static_cast<void *>(Sk_queue[pos])};

                timeline.add(entry);

                return EventHandlerResult::EVENT_CONSUMED;
            }
            else if (Sk_queue[pos]->get_index() == super_key_index) // if the index match and the superkey is already enabled
            {
                // We want to send the key pressed event to the superkey.
                Sk_queue[pos]->key_pressed();
                return EventHandlerResult::EVENT_CONSUMED;
            }
        }
    }
    else if (keyToggledOff(keyState))
    {
        for (uint8_t pos = 0; pos <= get_configured_sk(); ++pos)
        {
            if (Sk_queue[pos]->get_index() == super_key_index)
            {
                Sk_queue[pos]->key_released();
                return EventHandlerResult::EVENT_CONSUMED;
            }
        }
    }
    else if (keyIsPressed(keyState))
    {
        for (uint8_t pos = 0; pos <= get_configured_sk(); ++pos)
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

EventHandlerResult SuperkeysHandler::handle_regular_keys(Key &mapped_key, KeyAddr key_addr, uint8_t keyState)
{

    if (keyToggledOn(keyState))
    {
        if (ActionsDriver::isOnlyModifier(mapped_key))
        {
            save_pressed_modifiers(mapped_key, keyState);
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
        handle_regular_keys(mapped_key, key_addr, keyState);
        return EventHandlerResult::OK;
    }

    result = handle_superkeys(mapped_key, key_addr, keyState);

    return result;
}

EventHandlerResult SuperkeysHandler::beforeReportingState()
{
    // Iterate through every superkey if they are enabled.
    uint8_t configuredSK = get_configured_sk();

    for (uint8_t i = 0; i < configuredSK; i++)
    {
        if (Sk_queue[i]->is_enable())
        {
            Sk_queue[i]->run();
        }
    }

    NRF_LOG_FLUSH();

    return EventHandlerResult::OK;
}

EventHandlerResult SuperkeysHandler::onFocusEvent(const char *command)
{
    if (::Focus.handleHelp(command, "superkeys.map\nsuperkeys.waitfor\nsuperkeys.timeout\nsuperkeys.repeat\nsuperkeys.holdstart\nsuperkeys.overlap"))
        return EventHandlerResult::OK;

    if (strncmp_P(command, "superkeys.", 10) != 0) return EventHandlerResult::OK;

    // if (strcmp_P(command + 10, "map") == 0)
    // {
    //     if (::Focus.isEOL())
    //     {
    //         send_sk_map();
    //     }
    //     else
    //     {
    //         save_superkey_map();
    //     }
    // }
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
            uint16_t overlap_threshold = 0;
            ::Focus.read(overlap_threshold);
            configurations.overlap_threshold_ = overlap_threshold;
            save_configurations();
        }
    }

    return EventHandlerResult::EVENT_CONSUMED;
}

} // namespace plugin
} // namespace kaleidoscope
kaleidoscope::plugin::SuperkeysHandler superkeysHandler;

#pragma GCC pop_options