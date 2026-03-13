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

#ifndef KEY_ROLE_MANAGER_H
#define KEY_ROLE_MANAGER_H

#include "kbd_core.h"

#include "Kaleidoscope-FocusSerial.h"
#include "Kaleidoscope-Ranges.h"
#include "Superkeys/includes.h"
#include "Superkeys/Superkey/Superkey.h"
#include <Kaleidoscope.h>
#include <cstdint>

namespace kaleidoscope
{
namespace plugin
{

class KeyRoleManager : public kaleidoscope::Plugin
{
  public:

    typedef struct PACK
    {
        Superkey::superkey_config_t superkeys[Utils::SUPER_KEY_COUNT];
    } keyrole_config_t;

  public:

    struct modified_keys_t
    {
        uint16_t sk_id;
        uint16_t qukey_id;
        uint8_t flags_action_1;
        uint8_t flags_action_2;
    };
    
    KeyRoleManager();

    /**
     * Handle a key switch event and perform KeyRoleManager processing.
     *
     * This method is responsible for handling key switch events and performing KeyRoleManager processing when applicable.
     * It checks whether a key switch event corresponds to a physical key, ignores injected keys from other plugins,
     * and handles normal key events. It also manages the release of configured tap or held keys when modifiers are pressed or released.
     *
     * @param mapped_key The mapped key associated with the key switch event.
     * @param key_addr The address of the key for the key switch event.
     * @param keyState The state of the key switch event.
     * @return An EventHandlerResult indicating the result of the event handling.
     */
    EventHandlerResult onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t keyState);

    /**
     * Handle focus events and configuration commands for KeyRoleManager.
     *
     * This method processes focus events and configuration commands related to KeyRoleManager.
     * It allows you to manage and modify various settings for KeyRoleManager, such as key mappings, wait times, timeouts, hold durations, and overlap thresholds.
     *
     * @param command A character array representing the command for configuration.
     * @return An EventHandlerResult indicating the result of the event handling.
     */
    EventHandlerResult onFocusEvent(const char *command);

    /**
     * Prepare KeyRoleManager state before reporting key events.
     *
     * This method is responsible for managing the state of KeyRoleManager just before reporting key events
     *
     * @return An EventHandlerResult indicating the result of the operation.
     */
    EventHandlerResult beforeReportingState();

    EventHandlerResult onSetup();

    /**
     * Initialize KeyRoleManager with storage settings.
     *
     * This method sets up the KeyRoleManager by configuring storage parameters.
     * It allocates a storage slice in EEPROM to store the KeyRoleManager settings, including size and offset.
     * After setting up the storage, it updates the KeyRoleManager cache to ensure consistency with the stored values.
     */
    void setup_superkeys(void);

    Key search_and_replace(Key key);

    bool is_qukey(Key key);

    void get_superkey(Key* key);

    void setup_keys();

    /**
    * @brief Transforms the keymap bidirectionally between superkeys and qukeys
    * 
    * This function performs a two-pass transformation on the entire keymap stored in EEPROM:
    * 
    * Pass 1: Reverts all qukeys back to their original superkeys using the previous mapping.
    *         This is necessary because when superkey configurations change, the qukey codes
    *         they generate also change. We use previous_modified_keys[] which was saved
    *         before the configuration update to correctly identify and revert qukeys.
    * 
    * Pass 2: Transforms superkeys to qukeys based on the current configuration.
    *         Simple superkeys (tap + hold with modifier/layer) are converted to qukeys
    *         for performance optimization, while complex superkeys remain unchanged.
    * 
    * This approach ensures the keymap stays synchronized with superkey configuration changes,
    * handling both superkey→qukey and qukey→superkey transformations correctly.
    * 
    * @note This function is called after superkey configuration updates via Focus protocol
    * @note Changes are committed to EEPROM storage at the end
    */
    void transform_keymap_superkeys_to_qukeys();

    modified_keys_t* get_configured_qukeys(uint16_t qukey_id);

  private:

    const keyrole_config_t * p_keyrole_config = nullptr;

    // Current qukey mappings (superkey_id -> qukey_id)
    modified_keys_t modified_keys[Utils::MAX_SUPER_KEYS_ACTIVE];
    
    // Previous qukey mappings saved before configuration update
    // Used to correctly revert qukeys to superkeys when configurations change
    modified_keys_t previous_modified_keys[Utils::MAX_SUPER_KEYS_ACTIVE];

    uint8_t modified_keys_count;           // Number of active qukey mappings
    uint8_t previous_modified_keys_count;  // Number of previous qukey mappings
    
    uint8_t sk_index;
    uint8_t configured_superkeys;

    enum class key_roles_t
    {
        NONE,
        QUKEY,
        SUPERKEY
    };

    void determine_key_role();

//    void config();

    void send_sk_map();

    Key find_superkey_for_qukey(Key qukey);

    bool is_only_modifier(Key key);

    bool has_layer_change(Key key);

    void dumpKeymap();

    uint16_t setup_qukey();

    uint16_t replace_superkey_with_qukey(const Key *action_0, const Key *action_1);

    uint16_t calculate_qukey_code(uint32_t base_raw, uint16_t selected_keycode);


    void set_active_sk();

    void save_sk(const Key *action_0, const Key *action_1, const Key *action_2, const Key *action_3, const Key *action_4);

    void cfgmem_key_save( const Superkey::action_config_t * p_action_config, Superkey::action_config_t * p_action );
    void cfgmem_superkey_reset( const Superkey::superkey_config_t * p_superkey_config);
    void cfgmem_keyrole_reset( void );
};

} // namespace plugin
} // namespace kaleidoscope

extern kaleidoscope::plugin::KeyRoleManager keyRoleManager;

#endif // KEY_ROLE_MANAGER_H
