/* Kaleidoscope - Firmware for computer input devices
 * Copyright (C) 2013-2018  Keyboard.io, Inc.
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
#include "kaleidoscope/layers.h"
#include "kaleidoscope/keyswitch_state.h"

namespace kaleidoscope {

uint32_t Runtime_::millis_at_cycle_start_;

Runtime_::Runtime_(void) {
}

void
Runtime_::setup(void) {
  kaleidoscope::sketch_exploration::pluginsExploreSketch();
  kaleidoscope::Hooks::onSetup();

  device().setup();

  Layer.setup();
}

void
Runtime_::loop(void) {
  millis_at_cycle_start_ = millis();

  kaleidoscope::Hooks::beforeEachCycle();

  device().scanMatrix();

  kaleidoscope::Hooks::beforeReportingState();

  device().hid().keyboard().sendReport();
  device().hid().keyboard().releaseAllKeys();

  kaleidoscope::Hooks::afterEachCycle();
}

Runtime_ Runtime;

} // namespace kaleidoscope

kaleidoscope::Runtime_ &Kaleidoscope = kaleidoscope::Runtime;
