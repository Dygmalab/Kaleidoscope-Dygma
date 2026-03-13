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

#include "kbd_core.h"

#ifndef NRF_NEURON_SUPERKEYSHANDLER_H
#define NRF_NEURON_SUPERKEYSHANDLER_H

#include <Kaleidoscope.h>
#include <cstdint>
#include "Kaleidoscope-Ranges.h"
#include "Kaleidoscope-FocusSerial.h"

#include "Superkeys/includes.h"
#include "Superkeys/Superkey/Superkey.h"

using KeyID = uint16_t;

namespace kaleidoscope
{
    namespace plugin
    {
        class SuperkeysHandler : public kaleidoscope::Plugin
        {

        public:
            typedef struct PACK
            {
                // Superkey configurations
                uint16_t wait_for_;
                uint16_t hold_start_;
                uint8_t repeat_interval_;
                uint8_t overlap_threshold_;
                uint16_t time_out_;
            } superkey_config_t;

        public:
            // Kaleidoscope plugin methods

            /**
             * Handle a key switch event and perform SuperKeys processing.
             *
             * This method is responsible for handling key switch events and performing SuperKeys processing when applicable.
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
             * Handle focus events and configuration commands for SuperKeys.
             *
             * This method processes focus events and configuration commands related to DynamicSuperKeys.
             * It allows you to manage and modify various settings for SuperKeys, such as key mappings, wait times, timeouts, hold durations, and overlap thresholds.
             *
             * @param command A character array representing the command for configuration.
             * @return An EventHandlerResult indicating the result of the event handling.
             */
            EventHandlerResult onFocusEvent(const char *command);
            /**
             * Prepare DynamicSuperKeys state before reporting key events.
             *
             * This method is responsible for managing the state of SuperKeys just before reporting key events.
             * It handles releasing SuperKeys and applying time-outs if necessary.
             * Additionally, it ensures that fast key releases and the state of the keys are appropriately managed.
             *
             * @return An EventHandlerResult indicating the result of the operation.
             */
            EventHandlerResult beforeReportingState();
            /**
             * Initialize SuperKeys with storage settings.
             *
             * This method sets up the DynamicSuperKeys by configuring storage parameters.
             * It allocates a storage slice in EEPROM to store the DynamicSuperKeys settings, including size and offset.
             * After setting up the storage, it updates the SuperKeys cache to ensure consistency with the stored values.
             */
            static void setup(uint8_t active_superkeys, const Superkey::superkey_config_t * p_sk_map);

            /**
             * @brief Get the number of active superkeys.
             *
             * @return uint8_t The number of active superkeys.
             */
            static uint8_t get_configured_sk();

            static void save_superkey_map_from(const Superkey::superkey_config_t * p_sk_map, uint8_t active_superkeys);

            static void save_superkey_map();

            private:
            
            static const superkey_config_t * p_superkey_config;
            
            static uint8_t configured_superkeys;
            static uint8_t cache_modifiers;

            static void init(const Superkey::superkey_config_t * p_sk_map);
            static void config();
            static void enable();
            static void disable();
            static void refresh_configurations(const Superkey::superkey_config_t * p_sk_map);

            /*
             *  Erase superkeys instances to avoid memory leaks.
             */
            static void cleanup();

            static void save_pressed_modifiers(Key &mapped_key, uint8_t keyState);

            static EventHandlerResult handle_superkeys(Key &mapped_key, KeyAddr key_addr, uint8_t keyState);

            static EventHandlerResult handle_regular_keys(Key &mapped_key, KeyAddr key_addr, uint8_t keyState);

            static void cfgmem_wait_for_save( uint16_t wait_for );
            static void cfgmem_time_out_save( uint16_t time_out );
            static void cfgmem_hold_start_save( uint16_t hold_start );
            static void cfgmem_repeat_interval_save( uint8_t repeat_interval );
            static void cfgmem_overlap_threshold_save( uint8_t overlap_threshold );
            static void cfgmem_config_reset( void );
        };
    } // namespace plugin
} // namespace kaleidoscope
extern kaleidoscope::plugin::SuperkeysHandler superkeysHandler;

#endif // NRF_NEURON_SUPERKEYSHANDLER_H
