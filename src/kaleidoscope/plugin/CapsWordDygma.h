/* -*- mode: c++ -*-
 * Kaleidoscope-CapsWordDygma -- CapsWord plugin for Dygma keyboards
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

/* Keymap helper for the CapsWord toggle key. */
#define Key_CapsWord Key(kaleidoscope::ranges::CAPS_WORD)

namespace kaleidoscope {
namespace plugin {

/* CapsWord shifts every alpha key until the word ends (space, enter, tab,
 * punctuation, idle timeout, or pressing the key again).
 *
 * It is split into TWO plugins on purpose. Shifting a key is idempotent, but
 * toggling the feature on and off is not: every normal key press is delivered
 * twice by the SuperKeys timeline (Timeline::add() returns
 * check_interruptions() instead of an insertion flag), so a single plugin
 * placed after keyRoleManager would turn CapsWord on and straight back off in
 * the same cycle and it would never engage.
 *
 *   CapsWordTriggerDygma runs BEFORE keyRoleManager and owns the toggle, so it
 *   never sees the duplicate.
 *
 *   CapsWordShiftDygma runs LAST and only mutates keys and applies word-ending
 *   rules, all of which are idempotent, so the duplicate is harmless there.
 */
class CapsWordTriggerDygma : public kaleidoscope::Plugin {
 public:
  typedef struct __attribute__((packed))
  {
      /* Deactivate after this long without a word character. 0 = never. */
      uint16_t idle_timeout_ms;

      /* 0 disables the feature. */
      uint8_t  enabled;

      /* Which non-alpha keys continue the word instead of ending it. */
      uint8_t  continuation_flags;

  } capsword_config_t;

  /* continuation_flags bits */
  static constexpr uint8_t CONTINUE_DIGITS    = 0x01;
  static constexpr uint8_t CONTINUE_DASH      = 0x02;  /* '-' and '_' */
  static constexpr uint8_t CONTINUE_BACKSPACE = 0x04;

  static constexpr uint16_t DEFAULT_IDLE_TIMEOUT_MS = 5000;
  static constexpr uint16_t MAX_IDLE_TIMEOUT_MS     = 30000;
  static constexpr uint8_t  DEFAULT_CONTINUATION    = CONTINUE_DIGITS |
                                                      CONTINUE_DASH |
                                                      CONTINUE_BACKSPACE;

 public:
  CapsWordTriggerDygma(void) {}

  EventHandlerResult onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state);
  EventHandlerResult beforeReportingState();
  EventHandlerResult onFocusEvent(const char *command);

  /* Called from setup() after every other EEPROM slice has been requested, so
   * the new block lands at the end and no existing offset moves. */
  void reserveStorage();

  bool isActive(void) const { return active_; }

  void deactivate(void) { active_ = false; }

  /* Restart the idle timer; called for every character that continues the
   * word. Idempotent, so a duplicated delivery is harmless. */
  void noteActivity(void) { last_activity_ms_ = Runtime.millisAtCycleStart(); }

  uint8_t continuationFlags(void) const;

 private:
  bool     active_           = false;
  uint32_t last_activity_ms_ = 0;

  /* RAM copy of the EEPROM slice; p_capsword_config stays null until
   * reserveStorage() has loaded it. */
  capsword_config_t         config_;
  uint16_t                  storage_base_     = 0;
  const capsword_config_t * p_capsword_config = nullptr;

  void cfgmem_idle_timeout_save( uint16_t idle_timeout_ms );
  void cfgmem_enabled_save( uint8_t enabled );
  void cfgmem_continuation_save( uint8_t continuation_flags );
  void cfgmem_commit( void );
};

/* Registered LAST, so every plugin that matches keys by their exact raw value
 * sees Key_A rather than Key_A|SHIFT_HELD (raw 0x0804 instead of 0x0004).
 */
class CapsWordShiftDygma : public kaleidoscope::Plugin {
 public:
  CapsWordShiftDygma(void) {}

  EventHandlerResult onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state);
};

}
}

extern kaleidoscope::plugin::CapsWordTriggerDygma CapsWordTrigger;
extern kaleidoscope::plugin::CapsWordShiftDygma   CapsWordShift;
