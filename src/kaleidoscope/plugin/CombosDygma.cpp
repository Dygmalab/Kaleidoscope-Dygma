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

#include "CombosDygma.h"
#include "Kaleidoscope-EEPROM-Settings.h"

#include "Kaleidoscope-FocusSerial.h"
#include "kaleidoscope/key_events.h"
#include "kaleidoscope/keyswitch_state.h"
#include "kaleidoscope/layers.h"
#include "KeyRoleManager.h"
#include "Qukeys.h"

/* ---------------------------------------------------------------------------
 * TEMPORARY debug logging, same shape as the Autoshift and CapsWord plugins.
 * Set COMBOS_DEBUG_LOG to 0 to remove it.
 * ------------------------------------------------------------------------ */
#define COMBOS_DEBUG_LOG 0

#if COMBOS_DEBUG_LOG && defined(NRF52_ARCH)
  #define NRF_LOG_MODULE_NAME COMBOS
  #define NRF_LOG_LEVEL       4
  #include "nrf_log.h"
  NRF_LOG_MODULE_REGISTER();

  #define CB_LOG(...)  NRF_LOG_INFO(__VA_ARGS__)
#else
  #define CB_LOG(...)  do {} while (0)
#endif

namespace kaleidoscope {
namespace plugin {

/* ------------------------------------------------------------------------- */
/* Matching                                                                  */
/* ------------------------------------------------------------------------- */

bool CombosDygma::isEnabled(const combo_entry_t &combo) const
{
  if ((combo.flags & FLAG_ENABLED) == 0) return false;

  /* Defensive: a blob written by an older or buggy host could describe a combo
   * with fewer than two members, with a position that does not exist on this
   * board, or with the same position twice. Never trust the host -- a
   * one-member "combo" would fire on every single press of that key, and a
   * repeated position could never be completed, since the buffer holds each
   * address once. */
  uint8_t members = 0;

  for (uint8_t i = 0; i < MAX_MEMBERS; ++i)
  {
    if (combo.positions[i] == POSITION_UNUSED) continue;
    if (combo.positions[i] >= Runtime.device().numKeys()) return false;

    for (uint8_t j = 0; j < i; ++j)
    {
      if (combo.positions[j] == combo.positions[i]) return false;
    }

    members++;
  }

  if (members < MIN_MEMBERS) return false;

  return actionAllowed(combo.action);
}

/* Enabled, and bound to the layer the user is on right now. */
bool CombosDygma::isLive(const combo_entry_t &combo) const
{
  return isEnabled(combo) && comboAppliesToActiveLayer(combo);
}

bool CombosDygma::actionAllowed(uint16_t raw) const
{
  /* An Autoshift code as a combo action makes no sense: the combo has no
   * "hold the base key" gesture to build a shifted form from. */
  if (raw >= ranges::AUTOSHIFT_FIRST && raw <= ranges::AUTOSHIFT_LAST) return false;

  if (raw == 0) return false;

  return true;
}

/* A combo belongs to one layer and to no other: on any other layer its members
 * are ordinary keys, free to be members of that layer's own combos. Matching
 * against the most recent layer is what lets a layer key held a moment earlier
 * select which set of combos is live. */
bool CombosDygma::comboAppliesToActiveLayer(const combo_entry_t &combo) const
{
  return combo.layer == Layer.mostRecent();
}

bool CombosDygma::comboHasPosition(const combo_entry_t &combo, uint8_t offset) const
{
  for (uint8_t i = 0; i < MAX_MEMBERS; ++i)
  {
    if (combo.positions[i] == offset) return true;
  }

  return false;
}

uint8_t CombosDygma::memberCount(const combo_entry_t &combo) const
{
  uint8_t members = 0;

  for (uint8_t i = 0; i < MAX_MEMBERS; ++i)
  {
    if (combo.positions[i] != POSITION_UNUSED) members++;
  }

  return members;
}

bool CombosDygma::comboContainsBuffer(const combo_entry_t &combo) const
{
  for (uint8_t b = 0; b < buffer_len_; ++b)
  {
    if (!comboHasPosition(combo, buffer_[b].addr.toInt())) return false;
  }

  return true;
}

/* Can the key at a member position take part in a combo?
 *
 * Qukeys, Autoshift and CapsWord keys can. This plugin runs first in the
 * chain, so it decides before any of them sees the press: on a match their
 * plugin never sees the member at all, and on no match the buffered press is
 * re-emitted and reaches them as an ordinary press, just one match window
 * late. Two details make that hold, both in onKeyswitchEvent(): a release
 * inside the window is let through after the flush, and Qukeys replays are
 * never buffered.
 *
 * SuperKeys and OverlayKeys are still refused. The SuperKeys timeline counts
 * taps and interruptions against real press times and has not been validated
 * behind a buffer; an OverlayKey is not a key to type. Bazecor is supposed to
 * prevent assigning them, but an old blob would otherwise build a combo on top
 * of them.
 *
 * Checking the resolved key rather than the stored config is deliberate: it
 * follows layer changes, which a static check at write time cannot. */
bool CombosDygma::positionAllowed(KeyAddr key_addr, Key mapped_key) const
{
  uint16_t raw = mapped_key.getRaw();

  if (raw >= ranges::DYNAMIC_SUPER_FIRST && raw <= ranges::DYNAMIC_SUPER_LAST) return false; /* superkey */
  if (raw >= ranges::OVERLAY_KEY && raw <= ranges::OVERLAY_HOLD) return false;        /* overlay  */

  (void)key_addr;

  return true;
}

bool CombosDygma::isMemberPosition(uint8_t offset) const
{
  if (p_combos_config == nullptr) return false;

  uint8_t count = p_combos_config->combos_count;
  if (count > MAX_COMBOS) return false;

  for (uint8_t c = 0; c < count; ++c)
  {
    const combo_entry_t &combo = p_combos_config->combos[c];

    if (isLive(combo) && comboHasPosition(combo, offset)) return true;
  }

  return false;
}

/* Classifies the buffer against every live combo:
 *
 *   exact      -- index of a combo whose members are exactly the buffered keys,
 *                 or -1. With duplicates, the lowest index wins.
 *   extendable -- some combo contains every buffered key AND more, so another
 *                 press could still complete it.
 *
 * Neither: the buffer is a dead end. Both: an exact match exists but a longer
 * combo may still claim it, so the decision waits -- the longest combo wins,
 * which is what lets one key belong to a 2-key and a 3-key combo at once.
 *
 * The buffer holds at most MAX_MEMBERS and the list MAX_COMBOS, so a linear
 * scan is cheaper than any index would be. */
void CombosDygma::evaluateBuffer(int &exact, bool &extendable) const
{
  exact      = -1;
  extendable = false;

  if (p_combos_config == nullptr || buffer_len_ == 0) return;

  uint8_t count = p_combos_config->combos_count;
  if (count > MAX_COMBOS) return;

  for (uint8_t c = 0; c < count; ++c)
  {
    const combo_entry_t &combo = p_combos_config->combos[c];

    if (!isLive(combo) || !comboContainsBuffer(combo)) continue;

    if (memberCount(combo) == buffer_len_)
    {
      if (exact < 0) exact = (int)c;
    }
    else
    {
      extendable = true;
    }
  }
}

/* ------------------------------------------------------------------------- */
/* Idle requirement                                                          */
/* ------------------------------------------------------------------------- */

/* Does pressing this key mean the user is typing?
 *
 * Modifiers and layer shifts (isModifierKey, the same test Qukeys uses for its
 * SpaceCadet keys) and one-shot keys do not. They are exactly what a user
 * presses on purpose right before a combo: Ctrl held for Ctrl+<combo>, or the
 * layer key that selects which combos are live. */
bool CombosDygma::countsAsTyping(Key mapped_key)
{
  if (isModifierKey(mapped_key)) return false;

  uint16_t raw = mapped_key.getRaw();
  if (raw >= ranges::OS_FIRST && raw <= ranges::OS_LAST) return false;

  return true;
}

bool CombosDygma::idleRequirementMet() const
{
  uint16_t idle_time_ms = p_combos_config->idle_time_ms;

  if (idle_time_ms == 0 || !has_prior_press_) return true;

  return Runtime.hasTimeExpired(last_press_ms_, (uint32_t)idle_time_ms);
}

/* ------------------------------------------------------------------------- */
/* Buffer                                                                    */
/* ------------------------------------------------------------------------- */

bool CombosDygma::isBuffered(KeyAddr key_addr) const
{
  for (uint8_t i = 0; i < buffer_len_; ++i)
  {
    if (buffer_[i].addr == key_addr) return true;
  }

  return false;
}

void CombosDygma::bufferPress(KeyAddr key_addr, Key mapped_key)
{
  if (buffer_len_ >= MAX_MEMBERS) return;

  if (buffer_len_ == 0)
  {
    buffer_start_ = Runtime.millisAtCycleStart();
  }

  buffer_[buffer_len_].addr = key_addr;
  buffer_[buffer_len_].key  = mapped_key;
  buffer_len_++;

  state_ = State::COLLECTING;
}

void CombosDygma::clearBuffer()
{
  buffer_len_ = 0;
  state_      = State::IDLE;
}

/* The chord is over -- the window closed, a buffered key came up, or a key
 * that cannot extend it went down. Fire the combo the buffer matches exactly,
 * if there is one; otherwise the buffered presses were ordinary keys. */
void CombosDygma::resolveBuffer()
{
  int  exact      = -1;
  bool extendable = false;

  evaluateBuffer(exact, extendable);

  if (exact >= 0)
  {
    fireDeferred((uint8_t)exact);
  }
  else
  {
    flushBuffer();
  }
}

/* No match: let the buffered presses through as the ordinary keys they are.
 *
 * The stored Key is re-emitted rather than Key_NoKey. With Key_NoKey,
 * handleKeyswitchEvent re-resolves the address against the layer state at
 * FLUSH time, not at press time -- and a combo member may well be a layer key.
 * Qukeys re-emits its queued events the same way, for the same reason. */
void CombosDygma::flushBuffer()
{
  uint8_t       len = buffer_len_;
  BufferedPress pending[MAX_MEMBERS];

  for (uint8_t i = 0; i < len; ++i)
  {
    pending[i] = buffer_[i];
  }

  /* Clear before emitting: the flush re-enters this handler, and it must find
   * an empty buffer rather than the one it is draining. */
  clearBuffer();

  CB_LOG("[CB] no match, flushing %d buffered press(es)", len);

  flushing_ = true;

  for (uint8_t i = 0; i < len; ++i)
  {
    /* Mark BEFORE emitting. Qukeys will queue this event and replay it later
     * without INJECTED, and that replay must not be mistaken for a new press
     * -- otherwise the key is buffered again and never escapes. */
    addrFlagSet(flushed_, pending[i].addr, true);
    handleKeyswitchEvent(pending[i].key, pending[i].addr, IS_PRESSED);
  }

  flushing_ = false;
}

/* ------------------------------------------------------------------------- */
/* Active combo                                                              */
/* ------------------------------------------------------------------------- */

/* Bookkeeping shared by both ways a combo fires. The caller emits the action:
 * by rewriting the anchor's own event, or through fireDeferred(). */
void CombosDygma::activateCombo(uint8_t index, KeyAddr anchor)
{
  const combo_entry_t &combo = p_combos_config->combos[index];

  member_count_ = 0;

  for (uint8_t i = 0; i < MAX_MEMBERS; ++i)
  {
    if (combo.positions[i] == POSITION_UNUSED) continue;
    member_positions_[member_count_++] = combo.positions[i];
  }

  for (uint8_t i = 0; i < member_count_; ++i)
  {
    offsetFlagSet(owned_, member_positions_[i], true);
  }

  anchor_     = anchor;
  action_key_ = Key(combo.action);
  state_      = State::ACTIVE;

  buffer_len_ = 0;

  /* Not optional. key_events.cpp cached the member's own key for this address
   * before the plugin chain ran, so every downstream Key_NoKey re-lookup --
   * the Qukeys queue flush, its restore loop, the release path -- would
   * resolve the anchor back to the member key instead of the action. */
  Layer.updateLiveCompositeKeymap(anchor_, action_key_);

  CB_LOG("[CB] combo %d matched: %d members, anchor=%d action=0x%04X",
         index, member_count_, anchor_.toInt(), combo.action);
}

/* Fires a combo whose decision was held back while a longer combo could still
 * complete. There is no press left to rewrite, so the action is re-emitted at
 * the anchor's address exactly the way flushBuffer() re-emits an unmatched
 * press: a toggle-on WITHOUT INJECTED. SuperKeys and OneShot therefore see it
 * as a real keypress, which is what the rewrite path guarantees as well.
 *
 * The anchor is the last member pressed, as in the immediate path. From here
 * on the scanner's holds of the anchor are rewritten in the ACTIVE branch, and
 * its release ends the combo, both exactly as if the combo had fired on the
 * anchor's own press. */
void CombosDygma::fireDeferred(uint8_t index)
{
  KeyAddr anchor = buffer_[buffer_len_ - 1].addr;

  activateCombo(index, anchor);

  deferred_anchor_ = anchor_;
  deferred_action_ = action_key_;

  flushing_ = true;
  handleKeyswitchEvent(action_key_, anchor_, IS_PRESSED);
  flushing_ = false;
}

bool CombosDygma::isActiveMember(KeyAddr key_addr) const
{
  for (uint8_t i = 0; i < member_count_; ++i)
  {
    if (member_positions_[i] == key_addr.toInt()) return true;
  }

  return false;
}

void CombosDygma::endCombo(bool release_came_from_anchor)
{
  CB_LOG("[CB] combo released (from anchor=%d)", release_came_from_anchor);

  /* If the first member to come up is NOT the anchor, the anchor's own release
   * will never reach the chain -- we own that address and swallow it. Layer
   * keys need the toggle-off though: a ShiftToLayer action would stay stuck
   * on forever. So synthesize it.
   *
   * No INJECTED flag: handleKeyswitchEventDefault only calls releaseKey() for
   * injected events, and here we want Layer::eventHandler to see the toggle-off
   * while HID release happens by omission, exactly as for a physical release. */
  if (!release_came_from_anchor && anchor_.isValid())
  {
    state_ = State::IDLE;   /* stop owning the address before re-entering */
    flushing_ = true;
    handleKeyswitchEvent(action_key_, anchor_, WAS_PRESSED);
    flushing_ = false;
  }

  /* Note what does NOT happen here: ownership is not dropped, not even for an
   * anchor whose own toggle-off ended the combo. That release goes on to sit
   * in the Qukeys queue, and while it does, Qukeys' restore loop re-presses
   * the address once per cycle -- resolving, through the live composite
   * keymap, right back to the action. Staying owned is what keeps that
   * re-press silent. The bit is cleared by the next genuine toggle-on, in
   * onKeyswitchEvent. */

  state_        = State::IDLE;
  member_count_ = 0;
  anchor_       = KeyAddr();
  buffer_len_   = 0;
}

/* ------------------------------------------------------------------------- */
/* Event handling                                                            */
/* ------------------------------------------------------------------------- */

/* Has the user actually let go of the key?
 *
 * A toggle-off is not enough on its own. Qukeys swallows releases into its
 * queue and replays them later without the INJECTED flag, so the same physical
 * release arrives twice -- and the synthetic anchor release this plugin emits
 * comes back the same way. Treating a replay as "the user let go" drops
 * ownership of a key that is still physically down, which then starts typing
 * itself. */
bool CombosDygma::isPhysicalRelease(uint8_t key_state) const
{
  if (!keyToggledOff(key_state)) return false;

  return !keyRoleManager.isReplayingQueuedEvent();
}

EventHandlerResult CombosDygma::onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state)
{
  if (key_state & INJECTED) return EventHandlerResult::OK;

  /* Our own re-emissions must pass straight through. */
  if (flushing_) return EventHandlerResult::OK;

  if (!key_addr.isValid()) return EventHandlerResult::OK;

  if (p_combos_config == nullptr) return EventHandlerResult::OK;

  /* --- addresses we already released to the chain --------------------- */

  if (addrFlagGet(flushed_, key_addr))
  {
    if (isPhysicalRelease(key_state))
    {
      addrFlagSet(flushed_, key_addr, false);
    }

    /* Straight through: this is either the Qukeys replay of our own flush, or
     * the ordinary life of a key we already handed over. */
    return EventHandlerResult::OK;
  }

  /* --- members of a combo, still physically down ---------------------- */

  if (state_ != State::ACTIVE && addrFlagGet(owned_, key_addr))
  {
    /* The chord is over, but this address is not free yet.
     *
     * RELEASES always go through. Layer::eventHandler needs the toggle-off --
     * the anchor still resolves to the action through the live composite
     * keymap, so swallowing it would leave a ShiftToLayer action stuck on
     * forever. It costs nothing at HID level, where a non-injected toggle-off
     * is inert and release happens by omission.
     *
     * But a release does NOT end ownership, and this is the subtle part.
     * Letting it through hands it to Qukeys, which appends it to its queue and
     * returns EVENT_CONSUMED. Until that release is flushed, Qukeys' restore
     * loop in beforeReportingState re-presses the address every cycle with
     * `handleKeyswitchEvent(Key_NoKey, k, IS_PRESSED | WAS_PRESSED)` -- no
     * INJECTED flag, and Key_NoKey resolves through the live composite keymap
     * straight back to the member's own letter. Drop ownership on the release
     * and that re-press walks past this guard and types itself, with nothing
     * in our log to show for it.
     *
     * So ownership ends at the next GENUINE toggle-on and nowhere else: the
     * user pressing the key again. Qukeys' restore re-press is a hold, never a
     * toggle-on, and a replayed toggle-on is excluded explicitly -- for the
     * anchor, whose address resolves to the action, that replay would fire the
     * action a second time. */
    if (keyToggledOff(key_state))
    {
      return EventHandlerResult::OK;
    }

    /* The one replayed toggle-on that must go through: the action press of a
     * deferred combo that had already ended by the time Qukeys replayed it --
     * a chord tapped faster than the match window. It is the only press of
     * the action, so it is not a second firing; swallowing it would lose the
     * combo entirely. Its release is already queued behind it. */
    if (keyToggledOn(key_state) && keyRoleManager.isReplayingQueuedEvent() &&
        key_addr == deferred_anchor_)
    {
      deferred_anchor_ = KeyAddr();
      mapped_key       = deferred_action_;
      return EventHandlerResult::OK;
    }

    if (!keyToggledOn(key_state) || keyRoleManager.isReplayingQueuedEvent())
    {
      return EventHandlerResult::EVENT_CONSUMED;
    }

    /* A real new press. Release the address and handle it as any other. */
    addrFlagSet(owned_, key_addr, false);

    if (key_addr == deferred_anchor_) deferred_anchor_ = KeyAddr();
  }

  /* --- the idle requirement ------------------------------------------ */

  /* Sampled BEFORE this press is recorded: the question is whether the user
   * was idle up to now. Replays are excluded -- their physical press already
   * passed through here. */
  bool idle = true;

  if (keyToggledOn(key_state) && !keyRoleManager.isReplayingQueuedEvent())
  {
    idle = idleRequirementMet();

    if (countsAsTyping(mapped_key))
    {
      last_press_ms_   = Runtime.millisAtCycleStart();
      has_prior_press_ = true;
    }
  }

  /* --- a combo is currently held ------------------------------------- */

  if (state_ == State::ACTIVE)
  {
    if (isActiveMember(key_addr))
    {
      return handleActiveMember(mapped_key, key_addr, key_state);
    }

    /* Any other key behaves normally while the combo is held. */
    return EventHandlerResult::OK;
  }

  /* --- collecting ----------------------------------------------------- */

  /* A Qukeys replay is not a new press. Once Qukeys resolves a key it re-emits
   * it as a toggle-on WITHOUT the INJECTED flag -- the qukey as its primary or
   * alternate key, and whatever it queued behind it. By then the key may be
   * physically up and out of `flushed_`, and a member position that holds a
   * qukey on any layer resolves the replay to an ordinary key that passes
   * positionAllowed(). Buffering it would hold the resolution back one match
   * window and let the keys queued behind it overtake it: a home-row-mod roll
   * `d` `a` came out as `a` `d`, and a qukey hold took two hold timeouts to
   * produce its modifier. */
  if (keyToggledOn(key_state) && keyRoleManager.isReplayingQueuedEvent())
  {
    return EventHandlerResult::OK;
  }

  if (keyToggledOn(key_state))
  {
    if (!isMemberPosition(key_addr.toInt()) || !positionAllowed(key_addr, mapped_key))
    {
      /* A key that cannot be part of any combo went down. Whatever we were
       * collecting can no longer complete simultaneously, so resolve it now
       * and keep the output in press order. */
      if (state_ == State::COLLECTING)
      {
        resolveBuffer();
      }

      return EventHandlerResult::OK;
    }

    /* A chord only starts after the keyboard has been idle. Later members are
     * exempt: they are pressed within the match window by definition. */
    if (state_ == State::IDLE && !idle)
    {
      return EventHandlerResult::OK;
    }

    bufferPress(key_addr, mapped_key);

    int  exact      = -1;
    bool extendable = false;

    evaluateBuffer(exact, extendable);

    if (exact >= 0 && !extendable)
    {
      /* Complete, and nothing longer can claim it: fire on this very press.
       * Rewriting the event means everything downstream treats it as a real
       * press of the action at a real address. */
      activateCombo((uint8_t)exact, key_addr);
      mapped_key = action_key_;
      return EventHandlerResult::OK;
    }

    if (exact >= 0 || extendable)
    {
      /* Still growing -- or complete, but a longer combo could still take
       * it. The decision waits for the window, a release, or a key that
       * cannot extend the chord. */
      return EventHandlerResult::EVENT_CONSUMED;
    }

    /* This member belongs to a combo, but to none that contains what is
     * already buffered: the chord in progress ends here. Resolve it without
     * this press. The press itself then starts a new chord only if the idle
     * requirement still allows it -- normally it does not, since the keys
     * just resolved were pressed moments ago -- and otherwise goes out as an
     * ordinary key, after whatever the resolution emitted. */
    buffer_len_--;
    resolveBuffer();

    if (state_ == State::IDLE && idle)
    {
      bufferPress(key_addr, mapped_key);
      return EventHandlerResult::EVENT_CONSUMED;
    }

    return EventHandlerResult::OK;
  }

  /* Holds and releases of buffered members: keep swallowing the holds until the
   * buffer resolves one way or the other. A release means the chord is over,
   * so resolve what we have and then deal with the release itself.
   *
   * If the buffer was an exact match, the combo fires now and this release is
   * that of one of its members: it ends the combo exactly as it would have had
   * the combo fired on press.
   *
   * Otherwise the buffer was flushed, and the release must go through. The
   * scanner reports a toggle-off exactly once, so swallowing it would lose it
   * for good. An ordinary key does not notice -- the report is rebuilt every
   * cycle and the key drops out of it anyway -- but Qukeys and Autoshift decide
   * tap versus hold on that toggle-off. Without it a qukey tapped faster than
   * the window came out as its modifier, and an Autoshift key as the capital.
   * The press was re-emitted just above, so this is the physical release of a
   * flushed key: clear its mark here, as the flushed-key path would. */
  if (state_ == State::COLLECTING && isBuffered(key_addr))
  {
    if (keyToggledOff(key_state))
    {
      resolveBuffer();

      if (state_ == State::ACTIVE)
      {
        return handleActiveMember(mapped_key, key_addr, key_state);
      }

      addrFlagSet(flushed_, key_addr, false);
      return EventHandlerResult::OK;
    }

    return EventHandlerResult::EVENT_CONSUMED;
  }

  return EventHandlerResult::OK;
}

EventHandlerResult CombosDygma::handleActiveMember(Key &mapped_key, KeyAddr key_addr, uint8_t key_state)
{
  if (key_addr == anchor_)
  {
    if (keyToggledOff(key_state))
    {
      endCombo(true);
      /* Let the real toggle-off through, carrying the action, so
       * Layer::eventHandler sees it. */
      mapped_key = action_key_;
      return EventHandlerResult::OK;
    }

    /* The replay of a deferred action press, arriving while the combo is
     * still held: the usual case, nothing left pending. */
    if (keyToggledOn(key_state) && key_addr == deferred_anchor_)
    {
      deferred_anchor_ = KeyAddr();
    }

    /* Held: rewrite in place. The scanner re-reports the anchor every cycle,
     * which is what keeps the action pressed -- no re-injection loop needed. */
    mapped_key = action_key_;
    return EventHandlerResult::OK;
  }

  /* A non-anchor member. We own it: its physical holds, its release, and the
   * re-press that the Qukeys restore loop performs, must never reach the host.
   *
   * (Runtime.device().maskKey() would look like the tool for this, but it
   * unmasks AND continues on toggled_off, so the release would enter the
   * Qukeys queue and its restore loop would re-press the now-unmasked address
   * -- leaking the member key to the host for one cycle.) */
  if (keyToggledOff(key_state))
  {
    endCombo(false);
  }

  return EventHandlerResult::EVENT_CONSUMED;
}

EventHandlerResult CombosDygma::beforeReportingState()
{
  if (state_ != State::COLLECTING) return EventHandlerResult::OK;

  if (p_combos_config == nullptr) return EventHandlerResult::OK;

  if (Runtime.hasTimeExpired(buffer_start_, (uint32_t)p_combos_config->match_window_ms))
  {
    /* Window closed: fire the exact match that was waiting for a longer combo,
     * or flush. Combos runs before keyRoleManager, so this hook fires before
     * Qukeys' own -- the first re-emitted event is picked up in this same
     * cycle and no latency is added. */
    resolveBuffer();
  }

  return EventHandlerResult::OK;
}

/* ------------------------------------------------------------------------- */
/* Focus                                                                     */
/* ------------------------------------------------------------------------- */

EventHandlerResult CombosDygma::onFocusEvent(const char *command)
{
  if (::Focus.handleHelp(command, "combos.map\ncombos.window\ncombos.idleTime\ncombos.state"))
    return EventHandlerResult::OK;

  if (strncmp_P(command, "combos.", 7) != 0) return EventHandlerResult::OK;

  if (strcmp_P(command + 7, "map") == 0)
  {
    if (::Focus.isEOL())
    {
      ::Focus.send(p_combos_config->combos_count);

      for (uint8_t c = 0; c < MAX_COMBOS; ++c)
      {
        const combo_entry_t &combo = p_combos_config->combos[c];

        for (uint8_t i = 0; i < MAX_MEMBERS; ++i)
        {
          ::Focus.send(combo.positions[i]);
        }

        ::Focus.send(combo.layer);
        ::Focus.send(combo.flags);
        ::Focus.send(combo.action);
      }
    }
    else
    {
      uint8_t count = 0;
      ::Focus.read(count);

      if (count > MAX_COMBOS) count = MAX_COMBOS;

      for (uint8_t c = 0; c < MAX_COMBOS; ++c)
      {
        combo_entry_t entry;

        for (uint8_t i = 0; i < MAX_MEMBERS; ++i)
        {
          uint8_t position = POSITION_UNUSED;

          if (!::Focus.isEOL()) ::Focus.read(position);

          /* Clamp rather than trust: an out-of-range offset would index past
           * the key matrix during matching. */
          if (position != POSITION_UNUSED && position >= Runtime.device().numKeys())
          {
            position = POSITION_UNUSED;
          }

          entry.positions[i] = position;
        }

        /* Read into locals: Focus::read takes a reference, and a reference
         * cannot bind to a field of a packed struct. */
        uint8_t  layer  = 0;
        uint8_t  flags  = 0;
        uint16_t action = 0;

        if (!::Focus.isEOL()) ::Focus.read(layer);
        if (!::Focus.isEOL()) ::Focus.read(flags);
        if (!::Focus.isEOL()) ::Focus.read(action);

        if (!actionAllowed(action))
        {
          flags &= (uint8_t)~FLAG_ENABLED;
        }

        entry.layer  = layer;
        entry.flags  = flags;
        entry.action = action;

        cfgmem_combo_save(c, &entry);
      }

      cfgmem_count_save(count);

      /* A rewritten map invalidates anything in flight. */
      clearBuffer();
      state_        = State::IDLE;
      member_count_ = 0;
    }
  }

  if (strcmp_P(command + 7, "window") == 0)
  {
    if (::Focus.isEOL())
    {
      ::Focus.send(p_combos_config->match_window_ms);
    }
    else
    {
      uint16_t match_window_ms = 0;
      ::Focus.read(match_window_ms);

      if (match_window_ms < MIN_MATCH_WINDOW_MS) match_window_ms = MIN_MATCH_WINDOW_MS;
      if (match_window_ms > MAX_MATCH_WINDOW_MS) match_window_ms = MAX_MATCH_WINDOW_MS;

      cfgmem_match_window_save( match_window_ms );
    }
  }

  if (strcmp_P(command + 7, "idleTime") == 0)
  {
    if (::Focus.isEOL())
    {
      ::Focus.send(p_combos_config->idle_time_ms);
    }
    else
    {
      uint16_t idle_time_ms = 0;
      ::Focus.read(idle_time_ms);

      if (idle_time_ms > MAX_IDLE_TIME_MS) idle_time_ms = MAX_IDLE_TIME_MS;

      cfgmem_idle_time_save( idle_time_ms );
    }
  }

  /* Read-only diagnostics: count, window, state, buffered, active members,
   * idle time. */
  if (strcmp_P(command + 7, "state") == 0)
  {
    ::Focus.send(p_combos_config->combos_count);
    ::Focus.send(p_combos_config->match_window_ms);
    ::Focus.send((uint8_t)state_);
    ::Focus.send(buffer_len_);
    ::Focus.send(member_count_);
    ::Focus.send(p_combos_config->idle_time_ms);
  }

  return EventHandlerResult::EVENT_CONSUMED;
}

void CombosDygma::resetVolatileState()
{
  state_        = State::IDLE;
  buffer_len_   = 0;
  buffer_start_ = 0;
  member_count_ = 0;
  anchor_       = KeyAddr();
  flushing_     = false;

  last_press_ms_   = 0;
  has_prior_press_ = false;
  deferred_anchor_ = KeyAddr();

  for (uint8_t i = 0; i < sizeof(flushed_); ++i)
  {
    flushed_[i] = 0;
    owned_[i]   = 0;
  }
}

EventHandlerResult CombosDygma::onSetup()
{
  resetVolatileState();

  return EventHandlerResult::OK;
}

void CombosDygma::reserveStorage()
{
  storage_base_ = ::EEPROMSettings.requestAppendedSlice(sizeof(combos_config_t));
  Runtime.storage().get(storage_base_, config_);
  p_combos_config = &config_;

  /* A blob in another layout -- erased flash, or the 4-member layout of the
   * first version -- is wiped rather than misread: its entries would decode
   * into combos the user never configured. The window sits at the same offset
   * in both layouts and is kept if valid; the idle time overlaps the old first
   * entry and is not. Runs once: the version is stored last. */
  if( p_combos_config->layout_version != LAYOUT_VERSION )
  {
    combo_entry_t blank;

    for( uint8_t i = 0; i < MAX_MEMBERS; ++i )
    {
      blank.positions[i] = POSITION_UNUSED;
    }

    blank.layer  = 0;
    blank.flags  = 0;
    blank.action = 0;

    for( uint8_t c = 0; c < MAX_COMBOS; ++c )
    {
      cfgmem_combo_save( c, &blank );
    }

    cfgmem_count_save( 0 );
    cfgmem_idle_time_save( DEFAULT_IDLE_TIME_MS );
    cfgmem_layout_version_save( LAYOUT_VERSION );
  }

  /* Sanitize against the 0xFF pattern of erased flash. */
  if( p_combos_config->match_window_ms < MIN_MATCH_WINDOW_MS ||
      p_combos_config->match_window_ms > MAX_MATCH_WINDOW_MS )
  {
    cfgmem_match_window_save( DEFAULT_MATCH_WINDOW_MS );
  }

  if( p_combos_config->idle_time_ms > MAX_IDLE_TIME_MS )
  {
    cfgmem_idle_time_save( DEFAULT_IDLE_TIME_MS );
  }

  if( p_combos_config->combos_count > MAX_COMBOS )
  {
    cfgmem_count_save( 0 );
  }

  CB_LOG("reserveStorage: count=%d window=%d idle=%d",
         p_combos_config->combos_count, p_combos_config->match_window_ms,
         p_combos_config->idle_time_ms);
}

/****************************************************/
/*                   Config Memory                  */
/****************************************************/

void CombosDygma::cfgmem_match_window_save( uint16_t match_window_ms )
{
  config_.match_window_ms = match_window_ms;
  cfgmem_commit();
}

void CombosDygma::cfgmem_idle_time_save( uint16_t idle_time_ms )
{
  config_.idle_time_ms = idle_time_ms;
  cfgmem_commit();
}

void CombosDygma::cfgmem_layout_version_save( uint8_t version )
{
  config_.layout_version = version;
  cfgmem_commit();
}

void CombosDygma::cfgmem_count_save( uint8_t count )
{
  config_.combos_count = count;
  cfgmem_commit();
}

void CombosDygma::cfgmem_combo_save( uint8_t index, const combo_entry_t * p_entry )
{
  if( index >= MAX_COMBOS )
  {
    return;
  }

  config_.combos[index] = *p_entry;
  cfgmem_commit();
}

void CombosDygma::cfgmem_commit( void )
{
  Runtime.storage().put(storage_base_, config_);
  Runtime.storage().commit();
}

} // namespace plugin
} // namespace kaleidoscope

kaleidoscope::plugin::CombosDygma Combos;
