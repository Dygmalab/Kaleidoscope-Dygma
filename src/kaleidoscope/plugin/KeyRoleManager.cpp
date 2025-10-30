#include "KeyRoleManager.h"

#include "Kaleidoscope-Ranges.h"
#include "kaleidoscope/key_defs.h"

#include <Kaleidoscope-EEPROM-Keymap.h>
#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-FocusSerial.h>
#include <cstdint>

#include "Qukeys.h"
#include "SuperkeysHandler.h"
#include "kaleidoscope/plugin/Superkeys/Actions/ActionsDriver.h"
#include "kaleidoscope/plugin/Superkeys/includes.h"

#pragma GCC push_options
#pragma GCC optimize("O0")

namespace kaleidoscope
{
namespace plugin
{

uint8_t max_layers;
uint16_t settings_base_ = 0;

KeyRoleManager::KeyRoleManager()
{
    sk_index = 0;
    this->modified_keys_count = 0;
    // Initialize the modified_keys array
    for (auto & modified_key : modified_keys)
    {
        modified_key.qukey_id = 0;
        modified_key.sk_id = 0;
        modified_key.flags_action_1 = 0;
        modified_key.flags_action_2 = 0;
    }
}

/* HELPER FUNCTIONS */

auto is_idle = [](const Key &k) { return k.getRaw() == 1; };

static inline int layerIndexFromRaw(uint32_t raw)
{
    auto ranges_t = static_cast<Utils::KeyRanges>(ActionsDriver::find_key_type(raw));

    if(ranges_t != Utils::KeyRanges::LAYER_LOCK && ranges_t != Utils::KeyRanges::LAYER_SHIFT)
    {
        return -1;
    }
    else if (ranges_t == Utils::KeyRanges::LAYER_LOCK)
    {
        return static_cast<int>((raw - Utils::LAYER_LOCK_FIRST ) << 8); // múltiplos de 256   
    }
    else if (ranges_t == Utils::KeyRanges::LAYER_SHIFT)
    {
        return static_cast<int>((raw - Utils::LAYER_SHIFT_FIRST ) << 8); // múltiplos de 256   
    }
    return -1;
}

static inline int hidModToDumIndex(uint16_t hid)
{
    switch (hid)
    {
        case 0xE0:
        case 0xE4:
            return 0; // Ctrl
        case 0xE1:
        case 0xE5:
            return 1; // Shift
        case 0xE2:
            return 2; // Alt
        case 0xE3:
        case 0xE7:
            return 3; // OS/GUI
        case 0xE6:
            return 6; // AltGr
        default:
            return -1;
    }
}

bool KeyRoleManager::is_only_modifier(Key key)
{
    uint16_t key_id = key.getRaw() & 0x00FF; // We only take the HID keycode (lower part)

    // HID modifier range: 224 (0xE0) to 231 (0xE7)
    return (key_id >= 0xE0 && key_id <= 0xE7);
}

bool KeyRoleManager::has_layer_change(Key Action)
{
    auto ranges_t = static_cast<Utils::KeyRanges>(ActionsDriver::find_key_type(Action.getRaw()));
    switch (ranges_t)
    {
    case Utils::KeyRanges::LAYER_LOCK:
    case Utils::KeyRanges::LAYER_SHIFT:
    {
        return true;
    }
    break;
    
    default:
    {
        return false;
    }
    break;
    }
}

bool KeyRoleManager::is_qukey(Key key)
{
    bool result = false;

    if (key >= ranges::DUM_FIRST && key <= ranges::DUM_LAST) result = true;
    if (key >= ranges::DUL_FIRST && key <= ranges::DUL_LAST) result = true;

    return result;
}
/* END HELPER FUNCTIONS */

KeyRoleManager::modified_keys_t* KeyRoleManager::get_configured_qukeys(uint16_t qukey_id)
{
    for (modified_keys_t& key : this->modified_keys) 
    {
        if (key.qukey_id == qukey_id)
        {
            return &key;
        }    
    }
    return nullptr;
}

void KeyRoleManager::get_superkey(Key* mapped_key)
{
    for (modified_keys_t key : this->modified_keys) 
    {
        if (key.qukey_id == mapped_key->getRaw())
        {
            *mapped_key = Key(key.sk_id);
            return;
        }    
    }
}

uint16_t KeyRoleManager::calculate_qukey_code(uint32_t hold_action_raw, uint16_t tap_action_raw)
{
    const uint16_t tap_hid  = static_cast<uint16_t>(tap_action_raw  & 0x00FF);
    const uint16_t hold_hid = static_cast<uint16_t>(hold_action_raw & 0x00FF);

    // Case A: modifiers HID (Ctrl/Shift/Alt/OS/AltGr)
    if (hold_hid >= 0xE0 && hold_hid <= 0xE7) 
    {
        const int idx = hidModToDumIndex(hold_hid);
        if (idx < 0) return 0;
        const uint32_t base = ranges::DUM_FIRST + (static_cast<uint32_t>(idx) << 8);
        return static_cast<uint16_t>(base + tap_hid);
    }
    else 
    {
    // Case B: layer change (OSL / DUL)
        const uint32_t base = ranges::DUL_FIRST + layerIndexFromRaw(hold_action_raw);
        return static_cast<uint16_t>(base + tap_hid);
    }

    // Not a modifier or layer change
    return 0;
}

Key KeyRoleManager::search_and_replace(Key key)
{
    if (IS_OUTSIDE_DYNAMIC_SUPER_RANGE(key)) return key;

    uint8_t super_key_index = static_cast<uint8_t>(key.getRaw() - ranges::DYNAMIC_SUPER_FIRST);
      
    Key action_0 = key_storage.keys[super_key_index][0];
    Key action_1 = key_storage.keys[super_key_index][1];
    Key action_2 = key_storage.keys[super_key_index][2];
    Key action_3 = key_storage.keys[super_key_index][3];
    Key action_4 = key_storage.keys[super_key_index][4];

    if (!is_idle(action_0) && !is_idle(action_1) && is_idle(action_2) && is_idle(action_3) && is_idle(action_4))
    {
        // This is a fast Superkey, we need to check if it should be a Qukey or a Superkey,
        // The desition will depend if the action 1 is only a modifier.
        if (is_only_modifier(action_1)|| has_layer_change(action_1))
        {
            // QUKEY detected replacing it.
            uint16_t qukey_code = replace_superkey_with_qukey(&action_0, &action_1);
            return Key(qukey_code);
        }
        else
        {
            // SUPERKEY detected
            return key;
        }
    }
    else
    {
        // SUPERKEY found. Any other combination will be a normal superkey.
        return key;
    }    
}

uint16_t KeyRoleManager::replace_superkey_with_qukey(const Key *action_0, const Key *action_1)
{
    if (!action_0 || !action_1) return 0;

    const uint16_t tap_raw = static_cast<uint16_t>(action_0->getRaw());
    const uint16_t hold_raw = static_cast<uint16_t>(action_1->getRaw());

    const uint16_t qukey_code = calculate_qukey_code(hold_raw, tap_raw);
    if (qukey_code == 0u)
    {
        // ERROR: Quekey base not valid.
        return 0;
    }
    return qukey_code;
}

void KeyRoleManager::config()
{
    Runtime.storage().get(settings_base_, key_storage);

    // if one block is invalid, restart everything
    bool reset_storage = true;

    for (uint8_t i = 0; i < Utils::SUPER_KEY_COUNT; i++)
    {
        for (size_t j = 0; j < KEYS_IN_SUPERKEY; j++)
        {
            if (key_storage.keys[i][j] != 0xFFFF)
            {
                reset_storage = false;
            }
        }
    }

    if (reset_storage)
    {
        // Restart key_storage.
        key_storage.reset();
        Runtime.storage().put(settings_base_, key_storage);
        Runtime.storage().commit();
    }
    Runtime.storage().get(settings_base_, key_storage);
}

void KeyRoleManager::setup_keys()
{
    config();
    set_active_sk();
    determine_key_role();
    SuperkeysHandler::save_superkey_map_from(key_storage.keys, this->configured_superkeys);
}

void KeyRoleManager::save_configurations()
{
    Runtime.storage().put(settings_base_, key_storage);
    Runtime.storage().commit();
    setup_keys();
}

EventHandlerResult KeyRoleManager::onSetup()
{
    settings_base_ = kaleidoscope::plugin::EEPROMSettings::requestSlice(sizeof(KeyRoleManager::key_storage_t));
    config();
    set_active_sk();
    determine_key_role();
    return EventHandlerResult::OK;
}

void KeyRoleManager::setup_superkeys(uint8_t _max_layers)
{
    max_layers = _max_layers;
    qukeys.onSetup();         // Initialize the Qukeys plugin.
    SuperkeysHandler::setup(configured_superkeys,key_storage.keys); // Initialize the SuperkeysHandler plugin.
}

void KeyRoleManager::set_active_sk() 
{
    this->configured_superkeys = 0;
  
    for (uint16_t i = 0; i < Utils::SUPER_KEY_COUNT; ++i) 
    {
      uint8_t _undefined_actions = 0;
  
      for (int j = 0; j < KEYS_IN_SUPERKEY; ++j) 
      {
        if (key_storage.keys[i][j] == 0xFFFF) 
        {
          ++_undefined_actions;
        }
      }
  
      if (_undefined_actions == KEYS_IN_SUPERKEY) 
      {
        break; // First row completely empty -> end of configured superkeys.
      }
  
      ++this->configured_superkeys;
    }

    if(this->configured_superkeys > Utils::MAX_SUPER_KEYS_ACTIVE)
    {
        NRF_LOG_ERROR("Superkey count %i is greater than %i", this->configured_superkeys, Utils::MAX_SUPER_KEYS_ACTIVE);
        NRF_LOG_FLUSH();
        this->configured_superkeys = Utils::MAX_SUPER_KEYS_ACTIVE;
    }
  }

void KeyRoleManager::determine_key_role()
{
    for (size_t i = 0; i < this->configured_superkeys; i++)
    {
        Key action_0 = key_storage.keys[i][0];
        Key action_1 = key_storage.keys[i][1];
        Key action_2 = key_storage.keys[i][2];
        Key action_3 = key_storage.keys[i][3];
        Key action_4 = key_storage.keys[i][4];

        if (!is_idle(action_0) && !is_idle(action_1) && is_idle(action_2) && is_idle(action_3) && is_idle(action_4))
        {
            // This is a fast Superkey, we need to check if it should be a Qukey or a Superkey,
            // The desition will depend if the action 1 is only a modifier.

            Key tap_action = action_0;
            Key hold_action = action_1;
            // Clean the flags of these keys to correctly calculate the qukey value.
            // SHIFT + F = 49434 or 0xC04E
            tap_action.setFlags(0);
            hold_action.setFlags(0);

            if (is_only_modifier(hold_action) || has_layer_change(action_1))
            {
                // QUKEY
                uint16_t qukey_code = replace_superkey_with_qukey(&action_0, &action_1);
                if(qukey_code != 0 || (qukey_code < ranges::DUL_FIRST || qukey_code > ranges::DUL_LAST))
                {
                    // QUKEY DETECTED
                    // Here we save the qukey and superkey id for later use.
                    this->modified_keys[this->modified_keys_count].sk_id = ranges::DYNAMIC_SUPER_FIRST + i;
                    this->modified_keys[this->modified_keys_count].qukey_id = qukey_code;
                    this->modified_keys[this->modified_keys_count].flags_action_1 = action_0.getFlags();
                    this->modified_keys[this->modified_keys_count].flags_action_2 = action_1.getFlags();

                    this->modified_keys_count++;
                }
            }
        }
    }
    this->modified_keys_count = 0;
}

void KeyRoleManager::send_sk_map()
{
    Kaleidoscope.storage().get(settings_base_, key_storage);
    for (uint16_t i = 0; i < Utils::SUPER_KEY_COUNT; ++i)
    {
        for (int j = 0; j < KEYS_IN_SUPERKEY; ++j)
        {
            ::Focus.send(key_storage.keys[i][j]);
        }
    }
}

EventHandlerResult KeyRoleManager::onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t keyState)
{
    if (qukeys.onKeyswitchEvent(mapped_key, key_addr, keyState) == EventHandlerResult::EVENT_CONSUMED)
    {
        return EventHandlerResult::EVENT_CONSUMED;
    }
    return superkeysHandler.onKeyswitchEvent(mapped_key, key_addr, keyState);
}

EventHandlerResult KeyRoleManager::onFocusEvent(const char *command)
{
    EventHandlerResult result = EventHandlerResult::OK;

    if (qukeys.onFocusEvent(command) == EventHandlerResult::EVENT_CONSUMED)
    {
        return EventHandlerResult::EVENT_CONSUMED;
    }
    result = superkeysHandler.onFocusEvent(command);

    if (strncmp_P(command, "superkeys.", 10) != 0) return EventHandlerResult::OK;

    if (strcmp_P(command + 10, "map") == 0)
    {

        if (::Focus.isEOL())
        {
            send_sk_map();
        }
        else
        {
            uint16_t pos = 0;
            Key key;

            while (!::Focus.isEOL() && pos < Utils::SUPER_KEY_COUNT * KEYS_IN_SUPERKEY)
            {
                ::Focus.read(key);
                key_storage.keys[pos / KEYS_IN_SUPERKEY][pos % KEYS_IN_SUPERKEY] = key;
                pos++;
            }
            save_configurations();
        }
        result = EventHandlerResult::EVENT_CONSUMED;
    }

    return result;
}

EventHandlerResult KeyRoleManager::beforeReportingState()
{
    if (qukeys.beforeReportingState() == EventHandlerResult::EVENT_CONSUMED)
    {
        return EventHandlerResult::EVENT_CONSUMED;
    }
    return superkeysHandler.beforeReportingState();
}

} // namespace plugin
} // namespace kaleidoscope
kaleidoscope::plugin::KeyRoleManager keyRoleManager;
// #pragma GCC pop_options