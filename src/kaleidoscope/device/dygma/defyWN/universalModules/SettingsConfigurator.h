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

#pragma once

#ifdef ARDUINO_RASPBERRY_PI_PICO

#include "kaleidoscope/plugin.h"

namespace kaleidoscope {
namespace device {
namespace dygma {
namespace defyWN {

class SettingsConfigurator : public kaleidoscope::Plugin {
 public:
  EventHandlerResult onFocusEvent(const char *command);
  EventHandlerResult onSetup();

 private:
  struct KeyScannerSettings {
    uint32_t crc;
    uint32_t pull_up_config;
    uint32_t cpu_speed;
    uint32_t spi_speed_base;
    uint32_t spi_speed_variation;
    uint32_t pooling_rate_base;
    uint32_t pooling_rate_variation;
    uint8_t led_driver_enabled;
    uint8_t underGlow_enabled;
  };
  KeyScannerSettings left_settings;
  KeyScannerSettings right_settings;
  struct config {
    uint32_t validation;
    uint32_t cpuSpeed;
  } config_;
  uint16_t cpu_base_;
};


}  // namespace defyWN
}  // namespace dygma
}  // namespace device
}  // namespace kaleidoscope

extern kaleidoscope::device::dygma::defyWN::SettingsConfigurator SettingsConfigurator;

#endif
