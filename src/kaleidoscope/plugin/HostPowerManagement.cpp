/* -*- mode: c++ -*-
 * Kaleidoscope-HostPowerManagement -- Host power management support plugin.
 * Copyright (C) 2017, 2018  Keyboard.io, Inc.
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

#include "kaleidoscope/Runtime.h"
#include <Kaleidoscope-HostPowerManagement.h>
#include <Kaleidoscope-LEDControl.h>

#ifdef __AVR__
// This is a terrible hack until Arduino#6964 gets implemented.
// It makes the `_usbSuspendState` symbol available to us.
extern uint8_t _usbSuspendState;
#endif

namespace kaleidoscope {
namespace plugin {

bool HostPowerManagement::was_suspended_ = false;
#ifdef __AVR__
bool HostPowerManagement::initial_suspend_ = true;
#elif defined(ARDUINO_ARCH_SAMD)
uint16_t HostPowerManagement::suspend_timer = 0;
uint16_t HostPowerManagement::saved_fnum = 0;
uint8_t HostPowerManagement::fnum_counter = 0;
#endif

EventHandlerResult HostPowerManagement::beforeEachCycle() {

#ifdef __AVR__
  if ((_usbSuspendState & (1 << SUSPI))) {
    if (!initial_suspend_) {
      if (!was_suspended_) {
        was_suspended_ = true;
        hostPowerManagementEventHandler(Suspend);
      } else {
        hostPowerManagementEventHandler(Sleep);
      }
    }
  } else {
    if (initial_suspend_)
      initial_suspend_ = false;
    if (was_suspended_) {
      was_suspended_ = false;
      hostPowerManagementEventHandler(Resume);
    }
  }
#elif defined(ARDUINO_ARCH_SAMD)
  // Check the frame number to determine if we're suspended
  uint8_t delay = 50;
  if (Runtime.hasTimeExpired(suspend_timer, delay)) {
    if (saved_fnum == USB->DEVICE.FNUM.bit.FNUM) {
      if (!was_suspended_) {
        // If we have the same frame number for 20 * 50 milliseconds we're most likely suspended
        if (fnum_counter >= 20) {
          hostPowerManagementEventHandler(Suspend);
          was_suspended_ = true;
        } else {
          fnum_counter++;
        }
      }
    } else {
      if (was_suspended_) {
        hostPowerManagementEventHandler(Resume);
        was_suspended_ = false;
      }

      saved_fnum = USB->DEVICE.FNUM.bit.FNUM;
      fnum_counter = 0;
    }

    suspend_timer = Runtime.millisAtCycleStart();
  }
#endif

  return EventHandlerResult::OK;
}

}
}

__attribute__((weak)) void hostPowerManagementEventHandler(kaleidoscope::plugin::HostPowerManagement::Event event) {
}

kaleidoscope::plugin::HostPowerManagement HostPowerManagement;
