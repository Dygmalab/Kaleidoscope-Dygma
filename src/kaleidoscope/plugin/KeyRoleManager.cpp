#include "KeyRoleManager.h"

#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-EEPROM-Keymap.h>
#include <Kaleidoscope-FocusSerial.h>
#include "kaleidoscope/layers.h"

#include "SuperkeysHandler.h"
#include "Qukeys.h"

namespace kaleidoscope
{
    namespace plugin
    {
        KeyRoleManager::KeyRoleManager() {}

        void KeyRoleManager::setup_superkeys()
        {
            qukeys.onSetup();         // Initialize the Qukeys plugin.
            superkeysHandler.setup(); // Initialize the SuperkeysHandler plugin.
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

            // TODO: Tomar el control del comando para actualizar el keymap del plugin de EEPROM.
            // TODO: 1) Analizo lo que recibo del comando de Superkeys.
            /*TODO: 2) Si detecto que es una Qukey tengo que "transformar" un keycode de Superkey (50000>) a un keycode de Qukey (<50000), como lo hace BZ.
             *   Es decir tengo una tabla con los valores de los modificadores y sumo el valor de la key.
             */

            if (qukeys.onFocusEvent(command) == EventHandlerResult::EVENT_CONSUMED)
            {
                return EventHandlerResult::EVENT_CONSUMED;
            }
            result = superkeysHandler.onFocusEvent(command);

            if (strncmp_P(command, "superkeys.", 10) != 0)
                return EventHandlerResult::OK;

            if (strcmp_P(command + 10, "map") == 0)
            {
                // Aca vamos a checkear si la superkey recibida es realmente una superkey o una qukey.
                // Si es una qukey, la transformamos y la enviamos al plugin de qukeys.
                // Si es una superkey, la enviamos al plugin de superkeys.

                if (::Focus.isEOL())
                {
                    NRF_LOG_DEBUG("Sending superkey map");
                    superkeysHandler.send_sk_map();
                }
                else
                {
                    uint16_t pos = 0;
                    Key key;

                    while (!::Focus.isEOL() && pos < SK_COUNT * KEYS_IN_SUPERKEY)
                    {
                        ::Focus.read(key);
                        key_storage.keys[pos / KEYS_IN_SUPERKEY][pos % KEYS_IN_SUPERKEY] = key;
                        pos++;
                    }

                    //Print the matrix
                    for (size_t i = 0; i < SK_COUNT; i++)
                    {
                        for (size_t j = 0; j < KEYS_IN_SUPERKEY; j++)
                        {
                            NRF_LOG_DEBUG("%d ",key_storage.keys[i][j].getRaw());
                        }
                        NRF_LOG_DEBUG("\n");
                        NRF_LOG_FLUSH();
                    }
                    

                    superkeysHandler.save_superkey_map();
                    //  uint16_t pos = 0;
                    //  Key key;

                    // while (!::Focus.isEOL())
                    // {
                    //     ::Focus.read(key);
                    //     configurations.keys[pos / 6][pos % 6] = key;
                    //     pos++;
                    //     if (pos % 6 == 0)
                    //     {
                    //         pos = (pos / 6) * 6; // Reset pos to the next superkey
                    //     }
                    // }
                    // save_configurations();
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
kaleidoscope::plugin::KeyRoleManager KeyRoleManager;