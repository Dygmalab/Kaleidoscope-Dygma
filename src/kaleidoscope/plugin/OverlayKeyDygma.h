/* -*- mode: c++ -*-
 * Kaleidoscope-OverlayKeyDygma -- OVERLAY key plugin for Dygma keyboards
 * Copyright (C) 2026 Dygma Lab S.L. www.dygma.com
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

#include "kaleidoscope/Runtime.h"
#include <Kaleidoscope-Ranges.h>

namespace kaleidoscope {
namespace plugin {

class OverlayKeyDygma : public kaleidoscope::Plugin {
 public:
  OverlayKeyDygma(void) {}

  EventHandlerResult onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state);
  EventHandlerResult beforeReportingState();
  EventHandlerResult onLayerChange();

 private:
  bool initial_sent_ = false;

  void notifyLayerChange(uint8_t layer);
  void notifyOverlayEvent(uint8_t event_type);
  void notifyOverlayTapKey(uint8_t event_type);
  void notifyOverlayHoldKey(uint8_t event_type);

  // Tap / hold / double-tap state machine for the OVERLAY key
  enum class State : uint8_t { IDLE, PRESSED, FIRST_UP, HOLD_ACTIVE, DOUBLE_PRESSED };
  State    overlay_state_  = State::IDLE;
  uint32_t press_start_ms_ = 0;
  uint32_t release_ms_     = 0;

  static constexpr uint16_t HOLD_TIMEOUT_MS = 200;
  static constexpr uint16_t TAP_WINDOW_MS   = 100;
};

}
}

extern kaleidoscope::plugin::OverlayKeyDygma OverlayKey;
