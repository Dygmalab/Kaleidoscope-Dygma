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
#include "Defy_wireless.h"
#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-FocusSerial.h>
#include <Kaleidoscope-IdleLEDsDefy.h>
#include <Kaleidoscope-LEDControl.h>

namespace kaleidoscope
{
namespace plugin
{
bool IdleLEDsDefy::sleep_ = false;
IdleLEDsDefy::IdleTime IdleLEDsDefy::idle_time_limit;
uint32_t IdleLEDsDefy::start_time_wired = 0;
uint32_t IdleLEDsDefy::start_time_wireless = 0;
uint32_t IdleLEDsDefy::start_time_true_sleep = 0;
bool IdleLEDsDefy::idle_ = false; // Initialize with false

uint32_t IdleLEDsDefy::idleTimeoutSeconds(uint32_t time_in_ms)
{
    return time_in_ms / 1000;
}

EventHandlerResult IdleLEDsDefy::beforeEachCycle()
{
    if (idle_time_limit.wired_ == 0 || idle_time_limit.wireless_ == 0) return EventHandlerResult::OK;
    auto const &keyScanner = Runtime.device().keyScanner();
    auto isDefyLeftWired = keyScanner.leftSideWiredConnection();
    auto isDefyRightWired = keyScanner.rightSideWiredConnection();

    if (isDefyLeftWired && isDefyRightWired)
    {
        if (::LEDControl.isEnabled() && Runtime.hasTimeExpired(start_time_wired, idle_time_limit.wired_))
        {
            ::LEDControl.disable();
            idle_ = true;
        }
    }
    else
    {
        if (::LEDControl.isEnabled() && Runtime.hasTimeExpired(start_time_wireless, idle_time_limit.wireless_))
        {
            ::LEDControl.disable();
            idle_ = true;
            sleep_ = false;
            start_time_true_sleep = Runtime.millisAtCycleStart();
        }
    }
    if (idle_time_limit.true_sleep_activated_ && !::LEDControl.isEnabled() && !sleep_ && idle_time_limit.wired_ &&
        Runtime.hasTimeExpired(start_time_true_sleep, idle_time_limit.true_sleep_))
    {
        Communications_protocol::Packet p{};
        p.header.command = Communications_protocol::SLEEP;
        Communications.sendPacket(p);
        sleep_ = true;
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

    start_time_wired = Runtime.millisAtCycleStart();
    start_time_wireless = Runtime.millisAtCycleStart();
    start_time_true_sleep = Runtime.millisAtCycleStart();
    sleep_ = false;
    return EventHandlerResult::OK;
}

uint16_t PersistentIdleDefyLEDs::settings_base_;

EventHandlerResult PersistentIdleDefyLEDs::onSetup()
{
    Communications.callbacks.bind(CONNECTED, (
                                                 [this](const Packet &)
                                                 {
                                                     start_time_wired = Runtime.millisAtCycleStart();
                                                     start_time_wireless = Runtime.millisAtCycleStart();
                                                     ::LEDControl.enable();
                                                 }));
    settings_base_ = ::EEPROMSettings.requestSlice(sizeof(IdleTime));

    // If idleTime is max, assume that EEPROM is uninitialized, and store the
    // defaults.
    IdleTime idle_time;
    Runtime.storage().get(settings_base_, idle_time);
    if (idle_time.wired_ == 0xffffffff)
    {
        idle_time.true_sleep_activated_ = false;
        idle_time.true_sleep_ = true_sleep_time_limit_default;
        idle_time.wired_ = idle_time_limit_default;
        idle_time.wireless_ = idle_time_limit_default_wireless;
    }
    setIdleTimeoutSeconds(idle_time);
    Runtime.storage().get(settings_base_, idle_time_limit);

    return EventHandlerResult::OK;
}

void PersistentIdleDefyLEDs::setIdleTimeoutSeconds(const IdleTime &data)
{
    Runtime.storage().put(settings_base_, data);
    Runtime.storage().commit();
}

EventHandlerResult PersistentIdleDefyLEDs::onFocusEvent(const char *command)
{

    if (::Focus.handleHelp(command, "idleleds.true_sleep\nidleleds.true_sleep_time\nidleleds.time_limit\nidleleds.wireless")) return EventHandlerResult::OK;

    if (strncmp(command, "idleleds.", 9) != 0) return EventHandlerResult::OK;

    if (strcmp(command + 9, "true_sleep") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(idle_time_limit.true_sleep_activated_ ? 1 : 0);
        }
        else
        {
            uint8_t enabled;
            ::Focus.read(enabled);
            idle_time_limit.true_sleep_activated_ = enabled;
            setIdleTimeoutSeconds(idle_time_limit);
        }
    }

    if (strcmp(command + 9, "true_sleep_time") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(idleTimeoutSeconds(idle_time_limit.true_sleep_));
        }
        else
        {
            uint16_t true_sleep;
            ::Focus.read(true_sleep);
            idle_time_limit.true_sleep_ = true_sleep * 1000;
            setIdleTimeoutSeconds(idle_time_limit);
        }
    }

    if (strcmp(command + 9, "time_limit") == 0)
    {
        if (::Focus.isEOL())
        {
            ::Focus.send(idleTimeoutSeconds(idle_time_limit.wired_));
        }
        else
        {
            uint16_t idle_time;
            ::Focus.read(idle_time);
            idle_time_limit.wired_ = idle_time * 1000;
            setIdleTimeoutSeconds(idle_time_limit);
        }
    }

    if (strcmp(command + 9, "wireless") == 0)
    {

        if (::Focus.isEOL())
        {
            ::Focus.send(idleTimeoutSeconds(idle_time_limit.wireless_));
        }
        else
        {
            uint16_t idle_time_wireless;
            ::Focus.read(idle_time_wireless);
            idle_time_limit.wireless_ = idle_time_wireless * 1000;
            setIdleTimeoutSeconds(idle_time_limit);
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