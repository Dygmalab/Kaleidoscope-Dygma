/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::Defy -- Kaleidoscope device plugin for Dygma
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

namespace kaleidoscope {
namespace device {
namespace dygma {
namespace dygma_keyboards {

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
  void releaseAllKeys();

  HandSide this_device_;

 private:
  dygma_keyboards::key_data key_data_{};
  bool new_key_;

public:

public:
  const key_data &getKeyData() {
    new_key_ = false;
    return key_data_;
  }

  [[nodiscard]] bool newKey() const { return new_key_; }
};

}  // namespace dygma_keyboards
}  // namespace dygma
}  // namespace device
}  // namespace kaleidoscope
#endif
