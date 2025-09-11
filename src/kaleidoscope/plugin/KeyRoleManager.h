#ifndef KEY_ROLE_MANAGER_H
#define KEY_ROLE_MANAGER_H

#include <Kaleidoscope.h>
#include "Kaleidoscope-Ranges.h"
#include "EEPROM-Settings.h"
#include "Kaleidoscope-FocusSerial.h"

namespace kaleidoscope
{
    namespace plugin
    {

        class KeyRoleManager : public kaleidoscope::Plugin
        {
        public:
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

            /**
             * Initialize KeyRoleManager with storage settings.
             *
             * This method sets up the KeyRoleManager by configuring storage parameters.
             * It allocates a storage slice in EEPROM to store the KeyRoleManager settings, including size and offset.
             * After setting up the storage, it updates the KeyRoleManager cache to ensure consistency with the stored values.
             */
            static void setup_superkeys();

            static void setup_keymap(uint8_t max);

        private:
            static constexpr uint8_t SK_COUNT = kaleidoscope::ranges::DYNAMIC_SUPER_LAST - kaleidoscope::ranges::DYNAMIC_SUPER_FIRST + 2;
            static constexpr uint8_t KEYS_IN_SUPERKEY = 6;
            static constexpr uint8_t IDLE_KEY = 1;
            struct key_storage_t
            {
                // Keys configured in every superkey action.
                Key keys[SK_COUNT][KEYS_IN_SUPERKEY];

                void reset()
                {
                    for (uint8_t i = 0; i < SK_COUNT; ++i)
                    {
                        for (uint8_t j = 0; j < KEYS_IN_SUPERKEY; ++j)
                        {
                            keys[i][j] = IDLE_KEY;
                        }
                    }
                }
            };
            key_storage_t key_storage;
        };

    } // namespace plugin
} // namespace kaleidoscope

extern kaleidoscope::plugin::KeyRoleManager KeyRoleManager;

#endif // KEY_ROLE_MANAGER_H