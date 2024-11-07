/* -*- mode: c++ -*-
 * Kaleidoscope-Idle-LEDs -- Turn off the LEDs when the keyboard's idle
 * Copyright (C) 2018, 2019  Keyboard.io, Inc
 * Copyright (C) 2019  Dygma, Inc
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

#pragma once

#ifdef ARDUINO_ARCH_NRF52

#include "kaleidoscope/Runtime.h"


namespace kaleidoscope {
namespace plugin {

class IdleLEDsDefy : public kaleidoscope::Plugin
{
    public:
        IdleLEDsDefy(void) {}

        struct IdleTime
        {
            bool activate_keybsides_sleep;      // Activate/Deactivate put to sleep the keyboard sides [bool].
            uint32_t sides_sleep_idle_t_ms;     // Timeout to put to sleep the keyboard sides [ms].
            uint32_t leds_off_usb_idle_t_ms;    // Power off time for LEDs, when the n2 is in USB mode [ms].
            uint32_t leds_off_ble_idle_t_ms;    // Power off time for LEDs, when the n2 is in BLE mode [ms].
            uint32_t sides_sleep_idle_wired_t_ms; // Timeout to put to sleep the keyboard sides when wired [ms].
        };

        static IdleTime Power_save;
        static constexpr const uint32_t leds_off_usb_idle_t_ms_default = 600000;  // 600.000 ms = 10 minutes
        static constexpr const uint32_t leds_off_ble_idle_t_ms_default = 300000;  // 300.000 ms = 5 minutes
        static constexpr const uint32_t sides_sleep_idle_t_ms_default = 60000;    // 60.000 ms = 1 minutes
        
        static void save_power_save_settings(const IdleTime& data);
        static uint32_t ms_to_seconds(uint32_t time_in_ms);

        EventHandlerResult beforeEachCycle();
        EventHandlerResult onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state);

    private:
        static bool idle_;

    protected:
        static uint32_t start_time_wired;
        static uint32_t start_time_wireless;
        static uint32_t start_time_true_sleep;
        static uint32_t start_time_true_sleep_wired;
        static bool sleep_;
};

class PersistentIdleDefyLEDs : public IdleLEDsDefy
{
    public:
        EventHandlerResult onSetup();
        EventHandlerResult onFocusEvent(const char *command);
        static void save_power_save_settings(const IdleTime& data);

    private:
        static uint16_t settings_base_;
};

}   // plugin
}   // kaleidoscope

extern kaleidoscope::plugin::IdleLEDsDefy IdleLEDsDefy;
extern kaleidoscope::plugin::PersistentIdleDefyLEDs PersistentIdleDefyLEDs;

#endif