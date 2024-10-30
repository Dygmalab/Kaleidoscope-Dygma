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

#include <Arduino.h>
#include "Communications_protocol.h"

struct cRGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t w;
};

namespace kaleidoscope {
namespace device {
namespace dygma {
namespace defyWN {

#define LED_BANKS           11

#define LEDS_PER_HAND       88
#define LPH                 LEDS_PER_HAND
#define LEDS_PER_BANK       8
#define LED_BYTES_PER_BANK  (sizeof(cRGB) * LEDS_PER_BANK)

#define LED_RED_CHANNEL_MAX 254

typedef union {
  cRGB leds[LEDS_PER_HAND];
  byte bytes[LED_BANKS][LED_BYTES_PER_BANK];
} LEDData_t;

// return what bank the led is in
#define LED_TO_BANK(led) (led / LEDS_PER_BANK)

typedef union {
  uint8_t rows[5];
  uint64_t all;
} key_data;
using namespace Communications_protocol;
class Hand {
 public:
  explicit Hand(Communications_protocol::Devices device);
  ;

  Devices this_device_;
  LEDData_t led_data{};

 private:
  defyWN::key_data key_data_{};
  bool new_key_;
  bool online_{false};

 public:
  const key_data &getKeyData() {
    new_key_ = false;
    return key_data_;
  }

  [[nodiscard]] bool newKey() const {
    return new_key_;
  }
};

}  // namespace defyWN
}  // namespace dygma
}  // namespace device
}  // namespace kaleidoscope

#endif
