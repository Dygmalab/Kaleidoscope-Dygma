/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::Wired -- Kaleidoscope device plugin for Dygma Wired
 * Copyright (C) 2017-2019  Keyboard.io, Inc
 * Copyright (C) 2017-2020  Dygma Lab S.L.
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

#include <Arduino.h>
#include "Hand.h"
#include "Communications.h"

namespace kaleidoscope::device::dygma::defyWN {

Hand::Hand(Communications_protocol::Devices device)
  : this_device_(device) {

  auto keyScanFunction = [this](Packet packet) {
    if (packet.header.device == this_device_) {
      new_key_ = true;
      memcpy(key_data_.rows, packet.data, sizeof(key_data));
    }
  };
  Communications.callbacks.bind(HAS_KEYS, keyScanFunction);
}
}  // namespace kaleidoscope::device::dygma::defyWN
#endif
