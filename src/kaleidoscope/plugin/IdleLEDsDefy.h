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

class IdleLEDsDefy : public kaleidoscope::Plugin {
 public:
   IdleLEDsDefy(void) {}
    struct IdleTime {
    bool true_sleep_activated_;
    uint32_t true_sleep_;
    uint32_t wired_;
    uint32_t wireless_;
  };
  static IdleTime idle_time_limit;
  static constexpr const uint32_t idle_time_limit_default = 600000;
  static constexpr const uint32_t idle_time_limit_default_wireless = 300000;
  static constexpr const uint32_t true_sleep_time_limit_default = 60000;
  static void setIdleTimeoutSeconds(const IdleTime& data);
  static uint32_t idleTimeoutSeconds(uint32_t time_in_ms);

  EventHandlerResult beforeEachCycle();
  EventHandlerResult onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state);

 private:
  static bool idle_;

 protected:
   static uint32_t start_time_wired;
   static uint32_t start_time_wireless;
   static uint32_t start_time_true_sleep;
};

class PersistentIdleDefyLEDs : public IdleLEDsDefy
{
 public:
  EventHandlerResult onSetup();
  EventHandlerResult onFocusEvent(const char *command);
  static void setIdleTimeoutSeconds(const IdleTime& data);
 private:
  static uint16_t settings_base_;
};

}
}

extern kaleidoscope::plugin::IdleLEDsDefy IdleLEDsDefy;
extern kaleidoscope::plugin::PersistentIdleDefyLEDs PersistentIdleDefyLEDs;
#endif