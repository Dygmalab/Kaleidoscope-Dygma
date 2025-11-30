/* ActionsDriver - ActionsDriver support for Kaleidoscope.
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
#ifndef NRF_NEURON_ACTIONSDRIVER_H
#define NRF_NEURON_ACTIONSDRIVER_H
#include "Superkeys/includes.h"
#include "Superkeys/Superkey/Superkey.h"
#include <cstdint>

using EventFunction = uint8_t (*)();

class ActionsDriver
{
public:
    static Utils::ExtendedActions return_type(uint8_t tap_count, const Superkey::superkey_config_t * p_superkey);

    static bool action_handler(uint8_t tap_count, const Superkey::superkey_config_t *p_superkey, const Key &key, const KeyAddr &keyAddr);

    /*
     * 0xE0 (224) → Left Ctrl
        0xE1 (225) → Left Shift
        0xE2 (226) → Left Alt
        0xE3 (227) → Left GUI
        0xE4 (228) → Right Ctrl
        0xE5 (229) → Right Shift
        0xE6 (230) → Right Alt
        0xE7 (231) → Right GUI
     * */
    static bool isOnlyModifier(Key key);

    static void send_regular_key(const Key &key, const KeyAddr &keyAddr);

    static bool key_can_interrupt(const Key &Action);
    static bool key_can_interrupt(const Superkey::action_config_t &Action);

    static void send_modifier(const Key &key, const KeyAddr &key_addr);

    static uint16_t find_key_type(uint16_t value);

    static void send_modifiers_from_flags(uint8_t modif_flags, const KeyAddr &key_addr);

    static void send_key_twice(const Key &key, const KeyAddr &keyAddr);
};

#endif // NRF_NEURON_ACTIONSDRIVER_H
