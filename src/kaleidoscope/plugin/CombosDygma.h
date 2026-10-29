/* -*- mode: c++ -*-
 * Kaleidoscope-CombosDygma -- Combo keys for Dygma keyboards
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

/* Pressing several keys at once fires a single action.
 *
 * Three decisions shape the whole implementation:
 *
 * 1. Combos are indexed by PHYSICAL POSITION, not by the resolved key. Matching
 *    therefore does not depend on the keymap, which is what lets this plugin
 *    run first in the chain without interacting with the Qukeys/SuperKeys
 *    resolution at all. It also makes the host UI direct: you tick physical
 *    keys. Every combo belongs to exactly one layer and only matches while
 *    that layer is the most recent one, so the same keys can do different
 *    things on different layers.
 *
 * 2. The action is emitted as a NON-INJECTED event at the address of the last
 *    member pressed (the "anchor"). Injection would silently kill two action
 *    types: SuperKeys and OneShot both ignore INJECTED events. A non-injected
 *    event means the action flows through Qukeys, SuperKeys, DynamicMacros,
 *    OneShot and Layer::eventHandler exactly like a real keypress, with a
 *    valid KeyAddr -- so every action type works. When the combo completes on
 *    a press, that press itself is rewritten; when the decision is deferred
 *    (see 3), the action is re-emitted the same way a no-match flush is.
 *
 * 3. A key may belong to several combos, and the longest one wins. While the
 *    buffered keys could still grow into a longer combo, an exact match is
 *    held back; it fires when the match window closes, when a buffered key
 *    comes up, or when a key that cannot extend the chord goes down.
 */
class CombosDygma : public kaleidoscope::Plugin {
 public:
  static constexpr uint8_t MAX_COMBOS  = 32;
  static constexpr uint8_t MAX_MEMBERS = 6;
  static constexpr uint8_t MIN_MEMBERS = 2;

  /* 0xFF in a position slot means "unused". */
  static constexpr uint8_t POSITION_UNUSED = 0xFF;

  /* flags bits */
  static constexpr uint8_t FLAG_ENABLED = 0x01;

  static constexpr uint16_t DEFAULT_MATCH_WINDOW_MS = 10;
  static constexpr uint16_t MIN_MATCH_WINDOW_MS     = 5;
  static constexpr uint16_t MAX_MATCH_WINDOW_MS     = 500;

  /* How long the keyboard must have been idle before the first member of a
   * combo is pressed. Fast typing rolls over keys constantly; without this, a
   * roll that happens to hit two members inside the match window fires the
   * combo. 0 disables the requirement. */
  static constexpr uint16_t DEFAULT_IDLE_TIME_MS = 100;
  static constexpr uint16_t MAX_IDLE_TIME_MS     = 1000;

  /* Stored in `layout_version`. Bump it whenever combo_entry_t changes: a blob
   * in an older layout is wiped on boot rather than misread. Version 1 never
   * wrote this byte, so it reads back as the 0xFF erase pattern. */
  static constexpr uint8_t LAYOUT_VERSION = 2;

  typedef struct __attribute__((packed))
  {
      uint8_t  positions[MAX_MEMBERS];  /* KeyAddr::toInt(), 0xFF = unused   */
      uint8_t  layer;                   /* the only layer it matches on      */
      uint8_t  flags;                   /* bit0 enabled                      */
      uint16_t action;                  /* Key raw value                     */
  } combo_entry_t;                      /* 10 B                              */

  typedef struct __attribute__((packed))
  {
      uint16_t      match_window_ms;    /* default 10; 0xFFFF => auto-heal   */
      uint8_t       combos_count;       /* 0xFF => 0                         */
      uint8_t       layout_version;     /* != LAYOUT_VERSION => wipe         */
      uint16_t      idle_time_ms;       /* default 100; 0xFFFF => auto-heal  */
      combo_entry_t combos[MAX_COMBOS];
  } combos_config_t;                    /* 326 B                             */

 public:
  CombosDygma(void) {}

  EventHandlerResult onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state);
  EventHandlerResult beforeReportingState();
  EventHandlerResult onFocusEvent(const char *command);
  EventHandlerResult onSetup();

  /* Called from setup() after every other EEPROM slice has been requested, so
   * the new block lands at the end and no existing offset moves. */
  void reserveStorage();

 private:
  enum class State : uint8_t {
    IDLE,        /* nothing buffered                                   */
    COLLECTING,  /* some members down, waiting for a match or timeout  */
    ACTIVE,      /* a combo matched; the action is being held          */
  };

  struct BufferedPress {
    KeyAddr addr;
    Key     key;    /* the RESOLVED key, kept so a flush re-emits what the
                     * keymap said at press time rather than what it says at
                     * flush time -- a layer may have changed in between. */
  };

  State   state_ = State::IDLE;

  /* Invariant: every buffered address belongs to at least one live combo that
   * contains all of them, so the buffer never outgrows MAX_MEMBERS. */
  BufferedPress buffer_[MAX_MEMBERS];
  uint8_t       buffer_len_   = 0;
  uint32_t      buffer_start_ = 0;

  /* The idle requirement. Only a physical press of a key that types counts:
   * modifiers, layer shifts and one-shot keys are what a user holds or taps
   * right before a combo on purpose -- a combo on layer 1 has to work with the
   * layer key pressed a moment earlier. */
  uint32_t last_press_ms_    = 0;
  bool     has_prior_press_  = false;

  /* Active combo bookkeeping. */
  uint8_t member_positions_[MAX_MEMBERS];
  uint8_t member_count_ = 0;
  KeyAddr anchor_;
  Key     action_key_;

  /* The action press emitted by fireDeferred(), until Qukeys replays it. When
   * a release resolves the chord, the combo fires and ends in the same cycle,
   * so that replay arrives with the anchor already back under the `owned_`
   * guard -- which would swallow it, and the chord would produce nothing. */
  KeyAddr deferred_anchor_;
  Key     deferred_action_;

  /* Re-entrancy guard. It must cover the WHOLE flush loop and mean "do not
   * buffer", not merely "do not consume": when the Qukeys queue fills up it
   * calls processQueue() synchronously from inside onKeyswitchEvent, which
   * re-enters handleKeyswitchEvent for a DIFFERENT address while we are still
   * inside our own flush. */
  bool flushing_ = false;

  /* ----------------------------------------------------------------------
   * Address ownership, one bit per key.
   *
   * `flushed_`: keys this plugin has already released to the chain. Qukeys
   * swallows them into its queue and replays them later WITHOUT the INJECTED
   * flag, so a replay looks exactly like a fresh physical press -- and
   * re-buffering it would loop forever, with the key never reaching the host.
   * Marked keys pass straight through until they are physically released.
   *
   * `owned_`: members of a combo that fired. They stay owned until physically
   * released, even after the combo ends: the user is still holding the chord,
   * and those keys must not start typing themselves. This covers the anchor
   * too, whose address still resolves to the action through the live composite
   * keymap.
   * -------------------------------------------------------------------- */
  static constexpr uint8_t MAX_TRACKED_KEYS = 128;

  uint8_t flushed_[MAX_TRACKED_KEYS / 8] = {0};
  uint8_t owned_[MAX_TRACKED_KEYS / 8]   = {0};

  static bool addrFlagGet(const uint8_t *map, KeyAddr key_addr) {
    if (!key_addr.isValid()) return false;
    uint8_t offset = key_addr.toInt();
    if (offset >= MAX_TRACKED_KEYS) return false;
    return (map[offset >> 3] & (1 << (offset & 7))) != 0;
  }

  static void addrFlagSet(uint8_t *map, KeyAddr key_addr, bool value) {
    if (!key_addr.isValid()) return;
    uint8_t offset = key_addr.toInt();
    if (offset >= MAX_TRACKED_KEYS) return;
    if (value) {
      map[offset >> 3] |= (1 << (offset & 7));
    } else {
      map[offset >> 3] &= ~(1 << (offset & 7));
    }
  }

  static void offsetFlagSet(uint8_t *map, uint8_t offset, bool value) {
    if (offset >= MAX_TRACKED_KEYS) return;
    if (value) {
      map[offset >> 3] |= (1 << (offset & 7));
    } else {
      map[offset >> 3] &= ~(1 << (offset & 7));
    }
  }

  /* RAM copy of the EEPROM slice; p_combos_config stays null until
   * reserveStorage() has loaded it. */
  combos_config_t         config_;
  uint16_t                storage_base_   = 0;
  const combos_config_t * p_combos_config = nullptr;

  /* --- matching ------------------------------------------------------- */

  bool    isEnabled(const combo_entry_t &combo) const;
  bool    isLive(const combo_entry_t &combo) const;
  bool    comboAppliesToActiveLayer(const combo_entry_t &combo) const;
  bool    comboHasPosition(const combo_entry_t &combo, uint8_t offset) const;
  uint8_t memberCount(const combo_entry_t &combo) const;
  bool    comboContainsBuffer(const combo_entry_t &combo) const;
  bool    isMemberPosition(uint8_t offset) const;
  bool    positionAllowed(KeyAddr key_addr, Key mapped_key) const;
  bool    actionAllowed(uint16_t raw) const;
  void    evaluateBuffer(int &exact, bool &extendable) const;

  /* --- idle requirement ----------------------------------------------- */

  static bool countsAsTyping(Key mapped_key);
  bool        idleRequirementMet() const;

  /* --- buffer --------------------------------------------------------- */

  bool isBuffered(KeyAddr key_addr) const;
  void bufferPress(KeyAddr key_addr, Key mapped_key);
  void resolveBuffer();
  void flushBuffer();
  void clearBuffer();

  /* --- active combo --------------------------------------------------- */

  void activateCombo(uint8_t index, KeyAddr anchor);
  void fireDeferred(uint8_t index);
  bool isActiveMember(KeyAddr key_addr) const;
  void endCombo(bool release_came_from_anchor);
  EventHandlerResult handleActiveMember(Key &mapped_key, KeyAddr key_addr, uint8_t key_state);

  /* Puts every piece of volatile state back to power-on values. A real reboot
   * gets this for free from static initialisation; doing it explicitly makes
   * onSetup() idempotent, which is what the host test harness relies on to
   * keep tests independent of each other. */
  bool isPhysicalRelease(uint8_t key_state) const;

  void resetVolatileState();

  void cfgmem_match_window_save( uint16_t match_window_ms );
  void cfgmem_idle_time_save( uint16_t idle_time_ms );
  void cfgmem_layout_version_save( uint8_t version );
  void cfgmem_combo_save( uint8_t index, const combo_entry_t * p_entry );
  void cfgmem_count_save( uint8_t count );
  void cfgmem_commit( void );
};

}
}

extern kaleidoscope::plugin::CombosDygma Combos;
