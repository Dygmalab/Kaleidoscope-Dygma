/* KeyRoleManager - KeyRole support for Kaleidoscope.
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

#include "KeyRoleManager.h"

#include "Kaleidoscope-Ranges.h"
#include "kaleidoscope/key_defs.h"

#include <Kaleidoscope-EEPROM-Keymap.h>
#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-FocusSerial.h>
#include <cstdint>

#include "QukeysDygma.h"
#include "SuperkeysHandler.h"
#include "kaleidoscope/plugin/Superkeys/Actions/ActionsDriver.h"
#include "kaleidoscope/plugin/Superkeys/includes.h"

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
    auto key_type = ActionsDriver::find_key_type(raw);
    auto ranges_t = static_cast<Utils::KeyRanges>(key_type);
    int layer_idx = -1;
    uint32_t base = 0;

    if (ranges_t == Utils::KeyRanges::LAYER_LOCK)
    {
        base = Utils::LAYER_LOCK_FIRST;
        layer_idx = static_cast<int>(raw - base);
        //NRF_LOG_DEBUG("layerIndexFromRaw: LAYER_LOCK - raw=0x%08lX, base=0x%08lX, layer_idx=%d", 
        //              raw, base, layer_idx);
    }
    else if (ranges_t == Utils::KeyRanges::LAYER_SHIFT)
    {
        base = Utils::LAYER_SHIFT_FIRST;
        layer_idx = static_cast<int>(raw - base);
        //NRF_LOG_DEBUG("layerIndexFromRaw: LAYER_SHIFT - raw=0x%08lX, base=0x%08lX, layer_idx=%d", 
        //              raw, base, layer_idx);
    }
    else
    {
        //NRF_LOG_DEBUG("layerIndexFromRaw: Not a layer key - raw=0x%08lX, type=%d", 
        //              raw, key_type);
    }
    
    return layer_idx;
}

static inline int hidModToDumIndex(uint16_t hid)
{
    // Check for right modifiers (0x100 bit set)
    bool is_right = (hid & 0x100) != 0;
    uint8_t mod = hid & 0xFF;  // Get just the HID code
    
    switch (mod)
    {
        case 0xE0: // Left Control
            return is_right ? 4 : 0;  // Right Ctrl is index 4, Left is 0
        case 0xE1: // Left Shift
            return is_right ? 5 : 1;  // Right Shift is index 5, Left is 1
        case 0xE2: // Left Alt
            return is_right ? 6 : 2;  // Right Alt is index 6, Left is 2
        case 0xE3: // Left GUI/OS
            return is_right ? 7 : 3;  // Right GUI is index 7, Left is 3
        case 0xE4: // Right Control
            return 4;  // Always right Ctrl (0xE4 is right control)
        case 0xE5: // Right Shift
            return 5;  // Always right Shift (0xE5 is right shift)
        case 0xE6: // Right Alt (AltGr)
            return 6;  // Always right Alt
        case 0xE7: // Right GUI/OS
            return 7;  // Always right GUI
        default:
            return -1;
    }
}

bool KeyRoleManager::is_only_modifier(Key key)
{
    uint16_t key_id = key.getRaw() & 0x00FF; // We only take the HID keycode (lower part)
    uint16_t flags = key.getRaw() & 0xFF00;  // Get the flags part

    // HIDmodifier range: 224 (0xE0) to 231 (0xE7) for left modifiers
    // Right modifiers are in the same range but with the 0x100 bit set
    return (key_id >= 0xE0 && key_id <= 0xE7) || 
           ((key_id | 0x100) >= 0xEE0 && (key_id | 0x100) <= 0xEE7);
}

bool KeyRoleManager::has_layer_change(Key Action)
{
    auto raw = Action.getRaw();
    auto key_type = ActionsDriver::find_key_type(raw);
    auto ranges_t = static_cast<Utils::KeyRanges>(key_type);

    bool is_layer = (ranges_t == Utils::KeyRanges::LAYER_LOCK || 
                    ranges_t == Utils::KeyRanges::LAYER_SHIFT);
    
    // Debug output
    //NRF_LOG_DEBUG("has_layer_change: raw=0x%04X, type=%d, is_layer=%d", raw, key_type, is_layer);
    
    return is_layer;
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
    // Extract HID keycode (lower 8 bits) and flags (upper 8 bits)
    const uint16_t tap_hid   = static_cast<uint16_t>(tap_action_raw  & 0x00FF);
    const uint16_t tap_flags = static_cast<uint16_t>((tap_action_raw >> 8) & 0x00FF);
    const uint16_t hold_hid  = static_cast<uint16_t>(hold_action_raw & 0x01FF); // Include 9th bit for right modifiers

    //NRF_LOG_DEBUG("calculate_qukey_code: hold_raw=0x%08lX, tap_raw=0x%04X, hold_hid=0x%04X, tap_hid=0x%02X, tap_flags=0x%02X",
    //              hold_action_raw, tap_action_raw, hold_hid, tap_hid, tap_flags);

    // First check if this is a layer change key using the raw action
    // Case A: modifiers HID (Ctrl/Shift/Alt/OS/AltGr)
    if (!has_layer_change(Key(hold_action_raw)) && ( (hold_hid >= 0xE0 && hold_hid <= 0xE7) || (hold_hid >= 0x1E0 && hold_hid <= 0x1E7) ) )
    {
        const int idx = hidModToDumIndex(hold_hid);
        if (idx < 0) {
            //NRF_LOG_DEBUG("  Invalid modifier index for hold_hid=0x%04X", hold_hid);
            return 0;
        }
        
        // Calculate the base address for this modifier
        uint32_t base = ranges::DUM_FIRST + (static_cast<uint32_t>(idx) << 8);
        
        // If tap has flags, use the upper half of the range (128-255)
        // Otherwise use the lower half (0-127)
        uint16_t qukey_code;
        if (tap_flags != 0) {
            qukey_code = static_cast<uint16_t>(base + 128 + (tap_hid & 0x7F));
        } else {
            qukey_code = static_cast<uint16_t>(base + (tap_hid & 0x7F));
        }
        
        //NRF_LOG_DEBUG("  Modifier qukey - idx=%d, base=0x%04X, code=0x%04X", idx, base, qukey_code);
        return qukey_code;
    }
    // Case B: layer changes - check using the raw action
    if (has_layer_change(Key(hold_action_raw)))
    {
        // Get the layer index from the raw action
        int layer_idx = layerIndexFromRaw(hold_action_raw);
        if (layer_idx < 0) {
            //NRF_LOG_DEBUG("  Invalid layer index for hold_action_raw=0x%08lX", hold_action_raw);
            return 0;
        }
        
        // Calculate the base address for this layer
        // Each layer gets 256 possible keycodes (0-255)
        uint32_t base = ranges::DUL_FIRST + (static_cast<uint32_t>(layer_idx) << 8);
        
        // Calculate the final qukey code
        uint16_t qukey_code;
        if (tap_flags != 0) {
            // If tap has flags, use the upper half of the range (128-255)
            qukey_code = static_cast<uint16_t>(base + 128 + (tap_hid & 0x7F));
        } else {
            // Otherwise use the lower half (0-127)
            qukey_code = static_cast<uint16_t>(base + (tap_hid & 0x7F));
        }
        
        // Debug output
        //NRF_LOG_DEBUG("  Layer qukey - hold=0x%04X, tap=0x%02X, layer_idx=%d, base=0x%04X, code=0x%04X", 
        //              hold_hid, tap_hid, layer_idx, base, qukey_code);
        
        return qukey_code;
    }

    // Not a modifier or layer change
    //NRF_LOG_DEBUG("  Not a modifier or layer change");
    return 0;
}

Key KeyRoleManager::search_and_replace(Key key)
{
    // Case 1: It's a superkey - check if it should be transformed to qukey
    if (!IS_OUTSIDE_DYNAMIC_SUPER_RANGE(key))
    {
        uint8_t super_key_index = static_cast<uint8_t>(key.getRaw() - ranges::DYNAMIC_SUPER_FIRST);
          
        Key action_0 = key_storage.keys[super_key_index][0];
        Key action_1 = key_storage.keys[super_key_index][1];
        Key action_2 = key_storage.keys[super_key_index][2];
        Key action_3 = key_storage.keys[super_key_index][3];
        Key action_4 = key_storage.keys[super_key_index][4];

        if (!is_idle(action_0) && !is_idle(action_1) && is_idle(action_2) && is_idle(action_3) && is_idle(action_4))
        {
            // This is a fast Superkey, check if it should be a Qukey
            
            // Check if tap action is a Layer Lock BEFORE cleaning flags - these cannot be converted to qukeys
            bool tap_is_layer_lock = (action_0.getRaw() >= Utils::LAYER_LOCK_FIRST && 
                                      action_0.getRaw() <= Utils::LAYER_LOCK_LAST);
            
            Key tap_action = action_0;
            tap_action.setFlags(0);
            
            // Only convert to Qukey if:
            // 1. Tap action is not a layer lock
            // 2. Tap action is not just a modifier
            // 3. Hold action is a modifier or layer change
            if (!tap_is_layer_lock && 
                !is_only_modifier(tap_action) && 
                (is_only_modifier(action_1) || has_layer_change(action_1)))
            {
                // Transform to QUKEY (flags will be stored in modified_keys[] and restored when needed)
                uint16_t qukey_code = replace_superkey_with_qukey(&action_0, &action_1);
                //NRF_LOG_DEBUG("[SK->QK] SK:0x%04X tap:0x%04X(flags:0x%02X) -> QK:0x%04X", 
                //              key.getRaw(), action_0.getRaw(), action_0.getFlags(), qukey_code);
                return Key(qukey_code);
            }
        }
        
        // Keep as SUPERKEY
        return key;
    }
    
    // Case 2: It's a qukey - check if it should be reverted to superkey
    else if (is_qukey(key))
    {
        Key superkey_equivalent = find_superkey_for_qukey(key);
        
        if (superkey_equivalent.getRaw() != key.getRaw())
        {
            //NRF_LOG_INFO("QK->SK: 0x%04X -> 0x%04X", key.getRaw(), superkey_equivalent.getRaw());
            return superkey_equivalent;
        }
        
        // Keep as QUKEY
        return key;
    }
    
    // Case 3: Neither superkey nor qukey - return unchanged
    return key;
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
        //NRF_LOG_ERROR("Superkey count %i is greater than %i", this->configured_superkeys, Utils::MAX_SUPER_KEYS_ACTIVE);
        //NRF_LOG_FLUSH();
        this->configured_superkeys = Utils::MAX_SUPER_KEYS_ACTIVE;
    }
  }

void KeyRoleManager::determine_key_role()
{
    // Reset the count to rebuild the mapping
    this->modified_keys_count = 0;
    
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
            // The decision will depend if the action 1 is only a modifier.
            
            // Check if tap action is a Layer Lock BEFORE cleaning flags - these cannot be converted to qukeys
            bool tap_is_layer_lock = (action_0.getRaw() >= Utils::LAYER_LOCK_FIRST && 
                                      action_0.getRaw() <= Utils::LAYER_LOCK_LAST);

            Key tap_action = action_0;
            Key hold_action = action_1;
            // Clean the flags of these keys to correctly calculate the qukey value.
            // SHIFT + F = 49434 or 0xC04E
            tap_action.setFlags(0);
            hold_action.setFlags(0);

            if (!tap_is_layer_lock && (is_only_modifier(hold_action) || has_layer_change(action_1)))
            {
                // QUKEY - flags will be stored in modified_keys[] and restored when needed
                uint16_t qukey_code = replace_superkey_with_qukey(&action_0, &action_1);
                if(qukey_code != 0 || (qukey_code < ranges::DUL_FIRST || qukey_code > ranges::DUL_LAST))
                {
                    // QUKEY DETECTED
                    // Here we save the qukey and superkey id for later use.
                    // IMPORTANT: We store the original flags so they can be restored when the qukey is used
                    this->modified_keys[this->modified_keys_count].sk_id = ranges::DYNAMIC_SUPER_FIRST + i;
                    this->modified_keys[this->modified_keys_count].qukey_id = qukey_code;
                    this->modified_keys[this->modified_keys_count].flags_action_1 = action_0.getFlags();
                    this->modified_keys[this->modified_keys_count].flags_action_2 = action_1.getFlags();

                    //NRF_LOG_DEBUG("[DETERMINE] SK[%d]:0x%04X -> QK:0x%04X flags_a1:0x%02X stored at idx:%d", 
                    //              i, ranges::DYNAMIC_SUPER_FIRST + i, qukey_code, action_0.getFlags(), this->modified_keys_count);
                    this->modified_keys_count++;
                }
            }
            else if (tap_is_layer_lock)
            {
                //NRF_LOG_DEBUG("SK %d remains as SUPERKEY (tap: layer lock 0x%04X)", i, tap_action.getRaw());
            }
        }
    }
    
    ////NRF_LOG_DEBUG("determine_key_role: found %d qukeys", this->modified_keys_count);
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

/**
 * @brief Finds the superkey that would generate a given qukey (unused in current implementation)
 * 
 * This function is kept for potential future use but is not currently called.
 * The two-pass transformation approach in transform_keymap_superkeys_to_qukeys()
 * uses the previous_modified_keys[] mapping instead.
 * 
 * @param qukey The qukey to search for
 * @return Key The corresponding superkey if found, otherwise the original qukey
 */
Key KeyRoleManager::find_superkey_for_qukey(Key qukey)
{
    //NRF_LOG_DEBUG("Searching for superkey that generates qukey 0x%04X", qukey.getRaw());
    
    for (uint16_t i = 0; i < configured_superkeys; i++)
    {
        Key superkey = Key(ranges::DYNAMIC_SUPER_FIRST + i);
        
        Key action_0 = key_storage.keys[i][0];
        Key action_1 = key_storage.keys[i][1];
        Key action_2 = key_storage.keys[i][2];
        Key action_3 = key_storage.keys[i][3];
        Key action_4 = key_storage.keys[i][4];
        
        if (is_idle(action_0)) continue;
        
        // Only process simple superkeys (tap + hold actions)
        if (!is_idle(action_1) && is_idle(action_2) && is_idle(action_3) && is_idle(action_4))
        {
            Key tap_action = action_0;
            Key hold_action = action_1;
            tap_action.setFlags(0);
            hold_action.setFlags(0);
            
            uint16_t potential_qukey_code = 0;
            if (is_only_modifier(hold_action) || has_layer_change(action_1))
            {
                potential_qukey_code = replace_superkey_with_qukey(&action_0, &action_1);
            }
            
            if (potential_qukey_code == qukey.getRaw())
            {
                if (is_only_modifier(action_1) || has_layer_change(action_1))
                {
                    return qukey;  // Still a qukey
                }
                else
                {
                    //NRF_LOG_INFO("Reverting qukey 0x%04X to superkey 0x%04X", qukey.getRaw(), superkey.getRaw());
                    return superkey;  // Revert to superkey
                }
            }
        }
    }
    
    return qukey;  // Not found, keep as qukey
}

void KeyRoleManager::transform_keymap_superkeys_to_qukeys()
{
    uint16_t total_keys = static_cast<uint16_t>(Runtime.device().numKeys()) * max_layers;
    uint16_t keymap_base = EEPROMKeymap::keymap_base();
    uint16_t qk_to_sk = 0;
    uint16_t sk_to_qk = 0;
    
    //NRF_LOG_INFO("Transforming keymap: %d total keys", total_keys);
    
    // Pass 1: Revert ALL qukeys to superkeys using PREVIOUS modified_keys[] mapping
    //NRF_LOG_DEBUG("Pass 1: Reverting qukeys to superkeys using %d previous mappings", previous_modified_keys_count);
    for (uint16_t i = 0; i < total_keys; i++)
    {
        Key stored_key = Key(Runtime.storage().read(keymap_base + i * 2 + 1),
                             Runtime.storage().read(keymap_base + i * 2));
        
        if (is_qukey(stored_key))
        {
            // Search in PREVIOUS modified_keys[] for this qukey
            for (uint8_t j = 0; j < Utils::MAX_SUPER_KEYS_ACTIVE; j++)
            {
                if (previous_modified_keys[j].qukey_id == stored_key.getRaw())
                {
                    // Found it! Revert to superkey
                    Key superkey = Key(previous_modified_keys[j].sk_id);
                    EEPROMKeymap::updateKey(i, superkey);
                    qk_to_sk++;
                    //NRF_LOG_DEBUG("Reverted qukey 0x%04X to superkey 0x%04X at pos %d", stored_key.getRaw(), superkey.getRaw(), i);
                    break;
                }
            }
        }
    }
    
    // Pass 2: Transform superkeys to qukeys based on current configuration
    //NRF_LOG_DEBUG("Pass 2: Transforming superkeys to qukeys");
    for (uint16_t i = 0; i < total_keys; i++)
    {
        Key stored_key = Key(Runtime.storage().read(keymap_base + i * 2 + 1),
                             Runtime.storage().read(keymap_base + i * 2));
        
        if (!IS_OUTSIDE_DYNAMIC_SUPER_RANGE(stored_key))
        {
            Key transformed_key = search_and_replace(stored_key);
            if (transformed_key.getRaw() != stored_key.getRaw())
            {
                EEPROMKeymap::updateKey(i, transformed_key);
                sk_to_qk++;
            }
        }
    }
    
    // Commit changes to EEPROM
    Runtime.storage().commit();
    //NRF_LOG_INFO("Keymap transformation completed: %d QK->SK, %d SK->QK", qk_to_sk, sk_to_qk);
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
            
            // Save current modified_keys before updating configuration
            for (uint8_t i = 0; i < Utils::MAX_SUPER_KEYS_ACTIVE; i++)
            {
                previous_modified_keys[i] = modified_keys[i];
            }
            previous_modified_keys_count = modified_keys_count;
            //NRF_LOG_DEBUG("Saved %d previous qukey mappings", previous_modified_keys_count);
            
            // Update configuration and rebuild superkey handlers
            save_configurations();
            
            // Rebuild the qukey mappings based on new configuration
            set_active_sk();
            determine_key_role();
            //NRF_LOG_DEBUG("Rebuilt mappings: found %d qukeys", modified_keys_count);
            
            // Transform the keymap
            transform_keymap_superkeys_to_qukeys();
            //NRF_LOGLOG_INFO("Superkeys configuration and keymap transformation completed");
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
