/* -*- mode: c++ -*-
 * Kaleidoscope-AutoshiftDygma -- Autoshift key plugin for Dygma keyboards
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

/* Keymap helper: AS(Key_A) is the autoshift flavour of Key_A. */
#define AS(k) Key(kaleidoscope::ranges::AUTOSHIFT_FIRST + (k).getKeyCode())

namespace kaleidoscope {
namespace plugin {

/* An Autoshift key taps its base character and, when held past the timeout,
 * emits the shifted one. Keymap encoding is `ranges::AUTOSHIFT_FIRST + keyCode`
 * where keyCode is a bare HID code in AUTOSHIFT_KEYCODE_MIN..MAX.
 *
 * The plugin is registered BEFORE keyRoleManager and consumes its own range, so
 * these events never enter the Qukeys queue nor the SuperKeys timeline, which
 * keeps it clear of both delay stages and of the timeline's double delivery.
 * The cost of that placement is that Qukeys/SuperKeys cannot see the key
 * either -- see the hasPendingRoleKey() bypass in onKeyswitchEvent().
 */
class AutoshiftDygma : public kaleidoscope::Plugin {
 public:
  typedef struct __attribute__((packed))
  {
      /* How long the key must be held to produce the shifted character. */
      uint16_t hold_timeout_ms;

      /* 0 disables the feature; keys then always emit their base character. */
      uint8_t  enabled;

      uint8_t  reserved;

  } autoshift_config_t;

 public:
  AutoshiftDygma(void) {}

  EventHandlerResult onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state);
  EventHandlerResult beforeReportingState();
  EventHandlerResult onFocusEvent(const char *command);

  /* Called from setup() after every other EEPROM slice has been requested, so
   * the new block lands at the end and no existing offset moves. */
  void reserveStorage();

  /* Only keys with a distinct shifted form are eligible: letters, digits and
   * punctuation. Modifiers, F-keys, navigation and every SYNTHETIC/RESERVED
   * code (layers, macros, LED, mouse, overlay) are rejected. */
  static constexpr uint8_t AUTOSHIFT_KEYCODE_MIN = 0x04;
  static constexpr uint8_t AUTOSHIFT_KEYCODE_MAX = 0x38;

  static constexpr uint16_t DEFAULT_HOLD_TIMEOUT_MS = 175;
  static constexpr uint16_t MIN_HOLD_TIMEOUT_MS     = 50;
  static constexpr uint16_t MAX_HOLD_TIMEOUT_MS     = 2000;

 private:
  enum class State : uint8_t {
    IDLE,      /* nothing pending */
    WAITING,   /* key held, tap/hold not decided yet */
    HOLDING,   /* timeout elapsed, emitting the shifted form */
  };

  State    state_        = State::IDLE;
  KeyAddr  pending_addr_;   /* default-constructed to the invalid address */
  uint8_t  pending_kc_   = 0;
  uint32_t start_time_   = 0;

  /* Address whose tap was emitted by rollover while the key was still down.
   * Its events are swallowed until it is physically released. */
  KeyAddr  resolved_addr_;

  /* One bit per key address: "this key was let through as a plain key because
   * something was pending when it was pressed".
   *
   * The decision has to outlive the press event itself. Qukeys swallows the
   * key into its queue and replays it later WITHOUT the INJECTED flag, and its
   * isQukey() re-reads the raw keymap rather than the live composite cache --
   * so the replay arrives as a fresh toggle-on carrying the original
   * AUTOSHIFT_* code, at a moment when nothing is pending any more. Deciding
   * again there would retain the key and turn Ctrl+A into Ctrl+Shift+A.
   *
   * Sized for the largest Dygma matrix rather than the current one, so the
   * plugin is board-independent. Stale bits are harmless: every genuine
   * physical press re-decides. */
  static constexpr uint8_t MAX_TRACKED_KEYS = 128;
  uint8_t  bypassed_[MAX_TRACKED_KEYS / 8] = {0};

  bool isBypassed(KeyAddr key_addr) const {
    if (!key_addr.isValid()) return false;
    uint8_t offset = key_addr.toInt();
    if (offset >= MAX_TRACKED_KEYS) return false;
    return (bypassed_[offset >> 3] & (1 << (offset & 7))) != 0;
  }

  void setBypassed(KeyAddr key_addr, bool bypassed) {
    if (!key_addr.isValid()) return;
    uint8_t offset = key_addr.toInt();
    if (offset >= MAX_TRACKED_KEYS) return;
    if (bypassed) {
      bypassed_[offset >> 3] |= (1 << (offset & 7));
    } else {
      bypassed_[offset >> 3] &= ~(1 << (offset & 7));
    }
  }

  /* RAM copy of the EEPROM slice; p_autoshift_config stays null until
   * reserveStorage() has loaded it. */
  autoshift_config_t         config_;
  uint16_t                   storage_base_      = 0;
  const autoshift_config_t * p_autoshift_config = nullptr;

  /* Emits the unshifted character as a complete press/release pair within the
   * current cycle. */
  void emitTap(void);

  /* Adds to the report every modifier key that is physically held since
   * before this cycle, ahead of a mid-scan sendReport(). */
  void pressHeldModifiers(void);

  /* Adds to the report the modifiers of every active one-shot, sticky ones
   * included, ahead of a mid-scan sendReport(). */
  void pressOneShotModifiers(void);

  /* Rewrites the event as the plain base key and lets it flow down the chain
   * as an ordinary key press. */
  void passThroughAsPlainKey(Key &mapped_key, KeyAddr key_addr,
                             uint8_t keycode, uint8_t key_state);

  static bool isValidKeycode(uint8_t keycode) {
    return (keycode >= AUTOSHIFT_KEYCODE_MIN) && (keycode <= AUTOSHIFT_KEYCODE_MAX);
  }

  void cfgmem_hold_timeout_save( uint16_t hold_timeout_ms );
  void cfgmem_enabled_save( uint8_t enabled );
  void cfgmem_commit( void );

  /* ----------------------------------------------------------------------
   * Diagnostics
   *
   * These counters are reported by the `autoshift.state` Focus command. The
   * plugins have no usable logging channel of their own on the wired neuron,
   * so Focus is how we answer "did the firmware even see this key?".
   *
   * Press a key, then read autoshift.state: if `seen` did not move, the key
   * never reached the plugin and the problem is in the keymap or the firmware
   * build, not in the plugin logic.
   * -------------------------------------------------------------------- */
  uint16_t debug_seen_     = 0;  /* events with an AUTOSHIFT_* code       */
  uint16_t debug_taps_     = 0;  /* resolved as a tap                     */
  uint16_t debug_holds_    = 0;  /* resolved as a hold (shifted)          */
  uint16_t debug_bypassed_ = 0;  /* let through: qukey/superkey pending   */
  uint16_t debug_degraded_ = 0;  /* let through: invalid/disabled/capsword */
};

}
}

extern kaleidoscope::plugin::AutoshiftDygma Autoshift;
