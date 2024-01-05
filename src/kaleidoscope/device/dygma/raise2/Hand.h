/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::raise2 -- Kaleidoscope device plugin for raise2
 * Defy Copyright (C) 2017-2019  Keyboard.io, Inc Copyright (C) 2017-2019  Dygma
 * Lab S.L.
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

#pragma once

#include "Communications_protocol.h"
#include "common.h"

struct cRGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t w;
};

namespace kaleidoscope {
namespace device {
namespace dygma {
namespace raise2 {

#define LED_BANKS           11

#define LEDS_PER_HAND       88
#define LPH                 LEDS_PER_HAND
#define LEDS_PER_BANK       8
#define LED_BYTES_PER_BANK  (sizeof(cRGB) * LEDS_PER_BANK)

#define LED_RED_CHANNEL_MAX 229

typedef union {
  cRGB leds[LEDS_PER_HAND];
  uint8_t bytes[LED_BANKS][LED_BYTES_PER_BANK];
} LEDData_t;

typedef union {
  uint8_t rows[5];
  uint64_t all;
} key_data;
class Hand {
 public:
   enum HandSide {
       RIGHT,
       LEFT,
   };
  explicit Hand(HandSide side);
  void init();

  HandSide this_device_;
  LEDData_t led_data{};

 private:
  raise2::key_data key_data_{};
  bool new_key_;

public:

public:
  const key_data &getKeyData() {
    new_key_ = false;
    return key_data_;
  }

  [[nodiscard]] bool newKey() const { return new_key_; }
};

}  // namespace raise2
}  // namespace dygma
}  // namespace device
}  // namespace kaleidoscope
#endif