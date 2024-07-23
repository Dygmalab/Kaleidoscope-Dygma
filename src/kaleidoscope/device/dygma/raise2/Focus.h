/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::raise2 -- Kaleidoscope device plugin for Dygma raise2
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
#ifdef ARDUINO_ARCH_NRF52

#pragma once

#include "kaleidoscope/plugin.h"

namespace kaleidoscope {
namespace device {
namespace dygma {
namespace raise2 {

class Focus : public kaleidoscope::Plugin {
   public:
    void init();
    EventHandlerResult onFocusEvent(const char *command);
};

}  // namespace raise2
}  // namespace dygma
}  // namespace device
}  // namespace kaleidoscope

extern kaleidoscope::device::dygma::raise2::Focus Raise2Focus;
#endif