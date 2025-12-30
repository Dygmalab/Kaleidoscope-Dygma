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
  uint8_t rows[5];   // Original format for split keyboards (5 bytes)
  uint64_t all;      // For quick operations
} key_data;

typedef union {
  uint16_t rows[5];  // Extended format for regular keyboards with >8 columns (10 bytes)
  uint8_t bytes[10];
} key_data_extended;

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
  dygma_keyboards::key_data_extended key_data_extended_{};
  bool new_key_;
  bool use_extended_format_{false};

public:
  const key_data &getKeyData() {
    new_key_ = false;
    return key_data_;
  }
  
  const key_data_extended &getKeyDataExtended() {
    return key_data_extended_;
  }
  
  [[nodiscard]] bool isExtendedFormat() const { return use_extended_format_; }
  [[nodiscard]] bool newKey() const { return new_key_; }
};

}  // namespace dygma_keyboards
}  // namespace dygma
}  // namespace device
}  // namespace kaleidoscope
#endif
