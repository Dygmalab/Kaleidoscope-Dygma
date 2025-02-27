/* -*- mode: c++ -*-
 * Kaleidoscope-Idle-LEDs -- Turn off the LEDs when the keyboard's idle
 * Copyright (C) 2018, 2019, 2020  Keyboard.io, Inc
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


#ifdef ARDUINO_ARCH_NRF52

#include "Communications.h"

#include "KeyboardManager.h"

#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-FocusSerial.h>
#include <Kaleidoscope-IdleLEDsDefy.h>
#include <Kaleidoscope-LEDControl.h>

namespace kaleidoscope
{
namespace plugin
{

//Deep sleep flag
bool IdleLEDsDefy::sleep_ = false;
IdleLEDsDefy::IdleTime IdleLEDsDefy::Power_save;
uint32_t IdleLEDsDefy::start_time_wired = 0;
uint32_t IdleLEDsDefy::start_time_wireless = 0;
uint32_t IdleLEDsDefy::start_time_true_sleep = 0;
uint32_t IdleLEDsDefy::start_time_true_sleep_wired = 0;
bool IdleLEDsDefy::idle_ = false; // Initialize with false
bool IdleLEDsDefy::new_connection_ = false; // Initialize with false

uint32_t IdleLEDsDefy::ms_to_seconds(uint32_t time_in_ms)
{
    return time_in_ms / 1000;
}

void IdleLEDsDefy::reset_timers()
{
    start_time_wired = Runtime.millisAtCycleStart();
    start_time_wireless = Runtime.millisAtCycleStart();
    start_time_true_sleep = Runtime.millisAtCycleStart();
    start_time_true_sleep_wired = Runtime.millisAtCycleStart();
    sleep_ = false;
}

void IdleLEDsDefy::new_connection_set()
{
    new_connection_ = true;
}

EventHandlerResult IdleLEDsDefy::beforeEachCycle()
{
    auto const &keyScanner = Runtime.device().keyScanner();
    auto isDefyLeftWired = keyScanner.leftSideWiredConnection();
    auto isDefyRightWired = keyScanner.rightSideWiredConnection();

    if (isDefyLeftWired &&
        isDefyRightWired &&
        !ble_innited())
    {
        if (Power_save.leds_off_usb_idle_t_ms != 0 &&
            ::LEDControl.isEnabled() &&
            Runtime.hasTimeExpired(start_time_wired, Power_save.leds_off_usb_idle_t_ms))
        {
            ::LEDControl.disable();
            start_time_true_sleep_wired = Runtime.millisAtCycleStart();
            sleep_ = false;
            idle_ = true;
        }

        /* We force the sleep in wired mode. This is usefull when the user turn off the computer with the keyboard in wired mode*/
        if (!sleep_)
        {
            if(Runtime.millisAtCycleStart() - start_time_true_sleep_wired > sides_sleep_idle_wired_t_ms_default)
            {
                Communications_protocol::Packet p{};
                p.header.device = Communications_protocol::Devices::KEYSCANNER_DEFY_RIGHT;
                p.header.command = Communications_protocol::SLEEP;
                Communications.sendPacket(p);

                p.header.device = Communications_protocol::Devices::KEYSCANNER_DEFY_LEFT;
                p.header.command = Communications_protocol::SLEEP;
                Communications.sendPacket(p);
                sleep_ = true;

                start_time_true_sleep_wired = Runtime.millisAtCycleStart();
            }
        }
    }
    else
    {
        if (Power_save.leds_off_ble_idle_t_ms != 0
            && ::LEDControl.isEnabled() &&
            Runtime.hasTimeExpired(start_time_wireless, Power_save.leds_off_ble_idle_t_ms))
        {
            ::LEDControl.disable();
            idle_ = true;
            sleep_ = false;
            start_time_true_sleep = Runtime.millisAtCycleStart();
        }

        if (Power_save.activate_keybsides_sleep &&
            !::LEDControl.isEnabled() &&
            !sleep_ &&
            Runtime.hasTimeExpired(start_time_true_sleep, Power_save.sides_sleep_idle_t_ms))
        {
            Communications_protocol::Packet p{};
            p.header.command = Communications_protocol::SLEEP;
            Communications.sendPacket(p);
            sleep_ = true;
        }
    }


    return EventHandlerResult::OK;
}

EventHandlerResult IdleLEDsDefy::onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state)
{

    if (idle_)
    {
        ::LEDControl.enable();
        idle_ = false;
    }
    reset_timers();

    return EventHandlerResult::OK;
}

uint16_t PersistentIdleDefyLEDs::settings_base_;

EventHandlerResult PersistentIdleDefyLEDs::onSetup()
{
    Communications.callbacks.bind(CONNECTED, (
                                                 [this](const Packet &)
                                                 {
                                                     if( new_connection_ == true )
                                                     {
                                                         new_connection_ = false;
                                                         reset_timers();
                                                     }

                                                     ::LEDControl.enable();
                                                 }));

    settings_base_ = ::EEPROMSettings.requestSlice(sizeof(IdleTime));

    // If idleTime is max, assume that EEPROM is uninitialized, and store the defaults.
    IdleTime idle_time;
    Runtime.storage().get(settings_base_, idle_time);
    if (idle_time.leds_off_usb_idle_t_ms == 0xffffffff)
    {
        idle_time.activate_keybsides_sleep = false;
        idle_time.sides_sleep_idle_t_ms = sides_sleep_idle_t_ms_default;
        idle_time.leds_off_usb_idle_t_ms = leds_off_usb_idle_t_ms_default;
        idle_time.leds_off_ble_idle_t_ms = leds_off_ble_idle_t_ms_default;
    }
    save_power_save_settings(idle_time);
    Runtime.storage().get(settings_base_, Power_save);

    Power_save.sides_sleep_idle_wired_t_ms = sides_sleep_idle_wired_t_ms_default; // Default value for wired mode 20 minutes.

    return EventHandlerResult::OK;
}

void PersistentIdleDefyLEDs::save_power_save_settings(const IdleTime &data)
{
    Runtime.storage().put(settings_base_, data);
    Runtime.storage().commit();
}

EventHandlerResult PersistentIdleDefyLEDs::onFocusEvent(const char *command)
{
    /*
        idleleds.time_limit         --> Set power off time for LEDs, when the n2 is in USB mode [seconds].
        idleleds.wireless           --> Set power off time for LEDs, when the n2 is in BLE mode [seconds].
        idleleds.true_sleep         --> Activate/Deactivate put to sleep the keyboard sides [bool].
        idleleds.true_sleep_time    --> Set the time to put sleep the keyboard sides [seconds].
     */

    if (::Focus.handleHelp(command, "idleleds.true_sleep\nidleleds.true_sleep_time\nidleleds.time_limit\nidleleds.wireless"))
    {
        return EventHandlerResult::OK;
    }

    if (strncmp(command, "idleleds.", 9) != 0)
    {
        return EventHandlerResult::OK;
    }

    if (strcmp(command + 9, "true_sleep") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(Power_save.activate_keybsides_sleep ? 1 : 0);
        }
        else
        {
            uint8_t enabled;
            ::Focus.read(enabled);
            Power_save.activate_keybsides_sleep = enabled;
            save_power_save_settings(Power_save);
        }
    }

    if (strcmp(command + 9, "true_sleep_time") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(ms_to_seconds(Power_save.sides_sleep_idle_t_ms));
        }
        else
        {
            uint16_t true_sleep;
            ::Focus.read(true_sleep);
            Power_save.sides_sleep_idle_t_ms = true_sleep * 1000;  // Convert from seconds to ms.
            save_power_save_settings(Power_save);
        }
    }

    if (strcmp(command + 9, "time_limit") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(ms_to_seconds(Power_save.leds_off_usb_idle_t_ms));
        }
        else
        {
            uint16_t idle_time;
            ::Focus.read(idle_time);
            Power_save.leds_off_usb_idle_t_ms = idle_time * 1000;  // Convert from seconds to ms.
            save_power_save_settings(Power_save);
        }
    }

    if (strcmp(command + 9, "wireless") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(ms_to_seconds(Power_save.leds_off_ble_idle_t_ms));
        }
        else
        {
            uint16_t idle_time_wireless;
            ::Focus.read(idle_time_wireless);
            Power_save.leds_off_ble_idle_t_ms = idle_time_wireless * 1000;  // Convert from seconds to ms.
            save_power_save_settings(Power_save);
        }
        return EventHandlerResult::EVENT_CONSUMED;
    }

    return EventHandlerResult::EVENT_CONSUMED;
}

} // namespace plugin
} // namespace kaleidoscope

kaleidoscope::plugin::IdleLEDsDefy IdleLEDsDefy;
kaleidoscope::plugin::PersistentIdleDefyLEDs PersistentIdleDefyLEDs;

#endif
