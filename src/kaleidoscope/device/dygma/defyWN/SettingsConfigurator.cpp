/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::Wired -- Kaleidoscope device plugin for Dygma Wired
 * Copyright (C) 2017-2019  Keyboard.io, Inc
 * Copyright (C) 2017-2019  Dygma Lab S.L.
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

#ifdef ARDUINO_RASPBERRY_PI_PICO

#include "kaleidoscope/Runtime.h"
#include <Kaleidoscope-FocusSerial.h>
#include <pico/stdlib.h>
#include "SettingsConfigurator.h"
#include "Communications_protocol.h"
#include "Communications.h"
#include "kaleidoscope/plugin/EEPROM-Settings.h"

namespace kaleidoscope {
namespace device {
namespace dygma {
namespace defyWN {

#ifndef WIRED_FIRMWARE_VERSION
#define WIRED_FIRMWARE_VERSION "<unknown>"
#endif

EventHandlerResult SettingsConfigurator::onFocusEvent(const char *command) {
  if (::Focus.handleHelp(command,
                         PSTR("hardware.version\n"
                              "hardware.side_power\n"
                              "hardware.side_ver\n"
                              "hardware.keyscanInterval\n"
                              "hardware.firmware\n"
                              "hardware.chip_id\n")))
    return EventHandlerResult::OK;

  if (strncmp_P(command, PSTR("hardware."), 9) != 0)
    return EventHandlerResult::OK;

  if (strcmp_P(command + 9, PSTR("version")) == 0) {
    ::Focus.send("Dygma Wired");
    return EventHandlerResult::EVENT_CONSUMED;
  }

  if (strcmp_P(command + 9, PSTR("firmware")) == 0) {
    ::Focus.send(WIRED_FIRMWARE_VERSION);
    return EventHandlerResult::EVENT_CONSUMED;
  }

  if (strcmp_P(command + 9, PSTR("chip_id")) == 0) {
    ::Focus.send(Runtime.device().getChipID().c_str());
    return EventHandlerResult::EVENT_CONSUMED;
  }

  if (strcmp_P(command + 9, PSTR("side_ver")) == 0) {
    //    ::Focus.send(Runtime.device().side.leftVersion());
    //    ::Focus.send(Runtime.device().side.rightVersion());
    return EventHandlerResult::EVENT_CONSUMED;
  }

  if (strcmp_P(command + 9, "keyscanInterval") == 0) {
    if (::Focus.isEOL()) {

      ::Focus.send(Runtime.device().settings.keyscanInterval());

      return EventHandlerResult::EVENT_CONSUMED;
    } else {

      uint8_t keyscan;
      ::Focus.read(keyscan);
      Runtime.device().settings.keyscanInterval(keyscan);

      return EventHandlerResult::EVENT_CONSUMED;
    }
  }

  if (strcmp_P(command + 9, PSTR("side_power")) == 0) {
    if (::Focus.isEOL()) {
      ::Focus.send(Runtime.device().side.getPower());
      return EventHandlerResult::EVENT_CONSUMED;
    } else {
      uint8_t power = Runtime.serialPort().parseInt();
      Runtime.device().side.setPower(power);
      return EventHandlerResult::EVENT_CONSUMED;
    }
  }
  return EventHandlerResult::OK;
}

EventHandlerResult SettingsConfigurator::onSetup() {
  return EventHandlerResult::OK;
}

}  // namespace defyWN
}  // namespace dygma
}  // namespace device
}  // namespace kaleidoscope

kaleidoscope::device::dygma::defyWN::SettingsConfigurator SettingsConfigurator;

#endif
