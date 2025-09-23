#include "KeyRoleManager.h"

#include "kaleidoscope/layers.h"
#include <Kaleidoscope-EEPROM-Keymap.h>
#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-FocusSerial.h>

#include "Qukeys.h"
#include "SuperkeysHandler.h"

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
}

void KeyRoleManager::dumpKeymap()
{
    for (uint8_t layer = 0; layer < max_layers; layer++)
    {
        for (auto key_addr : KeyAddr::all())
        {
            Key k = EEPROMKeymap::getKey(layer, key_addr);

            NRF_LOG_DEBUG("%d", k.getRaw());

            NRF_LOG_FLUSH();
        }
    }
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

uint32_t KeyRoleManager::calculate_qukey_code(uint32_t hold_action_raw, uint16_t tap_action_raw)
{
    // 1) HID puro de la tecla TAP
    const uint16_t tap_hid = static_cast<uint16_t>(tap_action_raw & 0x00FF);
    const uint16_t hold_hid = static_cast<uint16_t>(hold_action_raw & 0x00FF);

    uint32_t base = 0;
    if (hold_action_raw >= 0xE0 && hold_action_raw <= 0xE7)
    {
        // 2) Mapear el HOLD (modificador HID 0xE0..0xE7) a base DUM (múltiplos de 256)
        const int idx = hidModToDumIndex(static_cast<uint16_t>(hold_action_raw));
        if (idx < 0) return 0u;
        base = ranges::DUM_FIRST + (static_cast<uint32_t>(idx) << 8);
    }
    else
    {
        // TODO: acá más adelante soportás capas Dual si el hold no es modificador HID.
        return 0u;
    }

    return base + static_cast<uint32_t>(tap_hid);
}

void KeyRoleManager::save_sk(const Key *a0, const Key *a1, const Key *a2, const Key *a3, const Key *a4)
{
    if (sk_index >= Utils::SUPER_KEY_COUNT)
    {
        NRF_LOG_DEBUG("ERROR: Superkey Index overflow (cap=%u)", (unsigned)Utils::SUPER_KEY_COUNT);
        return;
    }
    sk_storage.keys[sk_index][0] = *a0;
    sk_storage.keys[sk_index][1] = *a1;
    sk_storage.keys[sk_index][2] = *a2;
    sk_storage.keys[sk_index][3] = *a3;
    sk_storage.keys[sk_index][4] = *a4;
    ++sk_index;
}


bool KeyRoleManager::replace_superkey_with_qukey(const Key *action_0, const Key *action_1)
{
    if (!action_0 || !action_1) return false;

    const uint16_t tap_raw = static_cast<uint16_t>(action_0->getRaw());
    const uint16_t hold_raw = static_cast<uint16_t>(action_1->getRaw()); // esperado: 0xE0..0xE7

    const uint32_t qukey_code = calculate_qukey_code(hold_raw, tap_raw);
    if (qukey_code == 0u)
    {
        NRF_LOG_DEBUG("ERROR: Qukey base no válida (hold_raw=%u)", (unsigned)hold_raw);
        NRF_LOG_FLUSH();
        return false;
    }

    NRF_LOG_DEBUG("Qukey calculada: %lu (tap_hid=%u, hold_raw=%u)", (unsigned long)qukey_code, (unsigned)(tap_raw & 0x00FF), (unsigned)hold_raw);
    NRF_LOG_FLUSH();
    return true;
}


EventHandlerResult KeyRoleManager::onSetup()
{
    settings_base_ = kaleidoscope::plugin::EEPROMSettings::requestSlice(sizeof(KeyRoleManager::key_storage_t));
    config();
    return EventHandlerResult::OK;
}

void KeyRoleManager::setup_superkeys(uint8_t _max_layers)
{
    max_layers = _max_layers;
    qukeys.onSetup();         // Initialize the Qukeys plugin.
    superkeysHandler.setup(); // Initialize the SuperkeysHandler plugin.
}

bool KeyRoleManager::is_only_modifier(Key key)
{
    uint16_t key_id = key.getRaw() & 0x00FF; // We only take the HID keycode (lower part)

    // HID modifier range: 224 (0xE0) to 231 (0xE7)
    return (key_id >= 0xE0 && key_id <= 0xE7);
}

auto is_idle = [](const Key &k) { return k.getRaw() == 1; };

void KeyRoleManager::determine_key_role()
{
    key_roles_t result = key_roles_t::NONE;
    uint8_t idle_actions = 0;
    for (size_t i = 0; i < Utils::SUPER_KEY_COUNT; i++)
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
            if (is_only_modifier(action_1))
            {
                // QUKEY
                if (replace_superkey_with_qukey(&action_0, &action_1))
                {
                    NRF_LOG_DEBUG("Qukey DETECTED");
                    // TODO: guardar Qukey en la EEPROM.
                }
            }
            else
            {
                // SUPERKEY
                NRF_LOG_DEBUG("Superkey DETECTED");
                save_sk(&action_0, &action_1, &action_2, &action_3, &action_4);
                NRF_LOG_FLUSH();
            }
        }
        else if (is_idle(action_0) && is_idle(action_1) && is_idle(action_2) && is_idle(action_3) && is_idle(action_4))
        {
            // En este caso la tecla enviada en el superkey.map es todos 1 y no debe ser tratada.
            NRF_LOG_DEBUG("ERROR: No Key FOUND");
        }
        else
        {
            // SUPERKEY FOUND
            // Cualquier otra combinacion sera una superkey normal.
            NRF_LOG_DEBUG("Superkey DETECTED");
            save_sk(&action_0, &action_1, &action_2, &action_3, &action_4);
            NRF_LOG_FLUSH();
        }

        NRF_LOG_FLUSH();
    }
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
        NRF_LOG_DEBUG("Restarting key_storage");
        key_storage.reset();
        Runtime.storage().put(settings_base_, key_storage);
        Runtime.storage().commit();
    }
    Runtime.storage().get(settings_base_, key_storage);
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

    if (strncmp_P(command, "superkeys.", 10) != 0) return EventHandlerResult::OK;

    if (strcmp_P(command + 10, "map") == 0)
    {
        // Aca vamos a checkear si la superkey recibida es realmente una superkey o una qukey.
        // Si es una qukey, la transformamos y la enviamos al plugin de qukeys.
        // Si es una superkey, la enviamos al plugin de superkeys.

        if (::Focus.isEOL())
        {
            NRF_LOG_DEBUG("Sending superkey map");
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
            Runtime.storage().put(settings_base_, key_storage);
            Runtime.storage().commit();

            determine_key_role();

            superkeysHandler.save_superkey_map_from(sk_storage.keys, sk_index);

            this->sk_index = 0;
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
#pragma GCC pop_options