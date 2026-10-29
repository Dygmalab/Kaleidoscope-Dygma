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

#include "AutoshiftDygma.h"
#include "Kaleidoscope-EEPROM-Settings.h"

#include "CapsWordDygma.h"
#include "KeyRoleManager.h"
#include "OneShot.h"

#include "Kaleidoscope-FocusSerial.h"
#include "kaleidoscope/key_events.h"
#include "kaleidoscope/keyswitch_state.h"

/* ---------------------------------------------------------------------------
 * TEMPORARY debug logging.
 * ------------------------------------------------------------------------ */
#define AUTOSHIFT_DEBUG_LOG 0

#if AUTOSHIFT_DEBUG_LOG && defined(NRF52_ARCH)
  /* Register a log module of our own instead of falling back to the SDK's
   * default `app` module. Two reasons: the output is tagged (`<info> AUTOSHIFT:
   * ...`) so it is easy to pick out of the boot chatter, and registration
   * becomes explicit rather than relying on nrf_log_frontend.c having done it.
   *
   * This has to sit at file scope, outside any namespace. */
  #define NRF_LOG_MODULE_NAME AUTOSHIFT
  #define NRF_LOG_LEVEL       4        /* DEBUG, so INFO gets through */
  #include "nrf_log.h"
  NRF_LOG_MODULE_REGISTER();

  #define AS_LOG(...)  NRF_LOG_INFO(__VA_ARGS__)
#else
  #define AS_LOG(...)  do {} while (0)
#endif

namespace kaleidoscope {
namespace plugin {

EventHandlerResult AutoshiftDygma::onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state)
{
  /* Our own injections, and the SuperKeys timeline re-injections that re-enter
   * the hook chain from the top, must never be reprocessed. This is also what
   * makes emitTap() safe to call from inside this handler. */
  if (key_state & INJECTED)
  {
    return EventHandlerResult::OK;
  }

  /* A key whose tap was already emitted by rollover is still physically down.
   * Swallow everything from it until it comes back up, otherwise the scanner
   * would keep re-pressing the character every cycle and the host would repeat
   * it. */
  if (key_addr == resolved_addr_)
  {
    if (keyToggledOff(key_state))
    {
      resolved_addr_ = KeyAddr();
    }

    return EventHandlerResult::EVENT_CONSUMED;
  }

  /* A resolved hold is rewritten in place, cycle after cycle, straight off the
   * scanner's held events. No re-injection loop is needed: the single
   * toggle-on issued in beforeReportingState() already latched the shift into
   * modifier_flag_mask, and requestModifiers() keeps re-arming it. */
  if (state_ == State::HOLDING && key_addr == pending_addr_)
  {
    mapped_key = Key(pending_kc_, SHIFT_HELD);

    if (keyToggledOff(key_state))
    {
      state_ = State::IDLE;
    }

    return EventHandlerResult::OK;
  }

  /* Rollover: ANY other key going down resolves a pending autoshift as a tap
   * right away -- including another autoshift key, which would otherwise
   * overwrite pending_addr_ and swallow the first character. Because we run
   * before keyRoleManager, the tap is emitted -- with its own explicit
   * sendReport() -- before Qukeys ever queues the interrupting key, so the two
   * reach the host in the right order. */
  if (state_ == State::WAITING && keyToggledOn(key_state) && key_addr != pending_addr_)
  {
    /* The tapped key has not been physically released yet, so claim its
     * address until it is. */
    resolved_addr_ = pending_addr_;
    emitTap();
  }

  uint16_t raw = mapped_key.getRaw();

  if (raw < ranges::AUTOSHIFT_FIRST || raw > ranges::AUTOSHIFT_LAST)
  {
    return EventHandlerResult::OK;
  }

  uint8_t keycode = (uint8_t)(raw - ranges::AUTOSHIFT_FIRST);

  if (keyToggledOn(key_state))
  {
    debug_seen_++;
    AS_LOG("[AS] down raw=0x%04X kc=0x%02X addr=%d enabled=%d timeout=%d",
           raw, keycode, key_addr.toInt(),
           p_autoshift_config != nullptr ? p_autoshift_config->enabled : 0xFF,
           p_autoshift_config != nullptr ? p_autoshift_config->hold_timeout_ms : 0xFFFF);
  }

  /* Degrade to a plain key, with no delay, when:
   *  - the host stored a keycode that has no distinct shifted form,
   *  - the feature is switched off, or
   *  - CapsWord is active and would produce the same shifted character anyway
   *    (waiting out the hold timeout would only add latency for nothing).
   */
  if (!isValidKeycode(keycode) ||
      p_autoshift_config == nullptr ||
      p_autoshift_config->enabled == 0 ||
      CapsWordTrigger.isActive())
  {
    if (keyToggledOn(key_state))
    {
      debug_degraded_++;
      AS_LOG("[AS] degraded to plain: validKc=%d cfg=%d capsword=%d",
             isValidKeycode(keycode), p_autoshift_config != nullptr, CapsWordTrigger.isActive());
    }
    passThroughAsPlainKey(mapped_key, key_addr, keycode, key_state);
    return EventHandlerResult::OK;
  }

  /* Consuming the event here hides the key from the Qukeys chord detection and
   * from the SuperKeys interruption detection, and re-injecting it later does
   * not help because both filter INJECTED. With a qukey or superkey in flight
   * that would turn Ctrl+C into a bare 'c' followed by Ctrl.
   *
   * So while something is unresolved we let the key through untouched: no
   * timer, no retention. Holding it then yields Ctrl+C rather than
   * Ctrl+Shift+C, which is what you want while arming a shortcut.
   *
   * The decision is taken once, on the genuine physical press, and remembered
   * for the rest of that press. Qukeys replays what it queued as a fresh
   * toggle-on with no INJECTED flag and with the original AUTOSHIFT_* code
   * (its isQukey() re-reads the raw keymap, not the live composite cache), by
   * which time nothing is pending any more -- deciding again there would
   * retain the key after all. */
  if (keyToggledOn(key_state) && !keyRoleManager.isReplayingQueuedEvent())
  {
    setBypassed(key_addr, keyRoleManager.hasPendingRoleKey());
  }

  if (isBypassed(key_addr))
  {
    if (keyToggledOn(key_state))
    {
      debug_bypassed_++;
      AS_LOG("[AS] bypassed: a qukey or superkey is still pending");
    }
    passThroughAsPlainKey(mapped_key, key_addr, keycode, key_state);
    return EventHandlerResult::OK;
  }

  if (keyToggledOn(key_state))
  {
    pending_addr_ = key_addr;
    pending_kc_   = keycode;
    start_time_   = Runtime.millisAtCycleStart();
    state_        = State::WAITING;

    AS_LOG("[AS] retained, waiting for tap/hold decision");

    return EventHandlerResult::EVENT_CONSUMED;
  }

  if (state_ == State::WAITING && key_addr == pending_addr_)
  {
    if (keyToggledOff(key_state))
    {
      emitTap();
    }

    return EventHandlerResult::EVENT_CONSUMED;
  }

  /* An autoshift key we are not tracking (e.g. a second one pressed while the
   * first is still pending). Swallow it rather than leaking the raw range code
   * to the rest of the chain. */
  return EventHandlerResult::EVENT_CONSUMED;
}

EventHandlerResult AutoshiftDygma::beforeReportingState()
{
  if (state_ != State::WAITING)
  {
    return EventHandlerResult::OK;
  }

  if (!Runtime.hasTimeExpired(start_time_, (uint32_t)p_autoshift_config->hold_timeout_ms))
  {
    return EventHandlerResult::OK;
  }

  /* Exactly ONE real toggle-on. modifier_flag_mask is only updated on
   * toggled_on, so a Key carrying SHIFT_HELD that arrives as a plain hold
   * would never put the shift in the report. From here on the key is kept
   * pressed by rewriting the scanner's held events in onKeyswitchEvent(). */
  handleKeyswitchEvent(Key(pending_kc_, SHIFT_HELD), pending_addr_, IS_PRESSED | INJECTED);

  debug_holds_++;
  AS_LOG("[AS] HOLD -> emitting shifted kc=0x%02X", pending_kc_);
  state_ = State::HOLDING;

  return EventHandlerResult::OK;
}

void AutoshiftDygma::passThroughAsPlainKey(Key &mapped_key, KeyAddr key_addr,
                                           uint8_t keycode, uint8_t key_state)
{
  mapped_key = Key(keycode, 0);

  /* key_events.cpp already cached the raw AUTOSHIFT_* code for this address
   * before the plugin chain ran, so every downstream Key_NoKey re-lookup --
   * the Qukeys queue flush and its restore loop, most of all -- would resolve
   * the address back to the range code and hand it to us again. Overwrite the
   * cache with what we actually emitted. */
  if (keyToggledOn(key_state))
  {
    Layer.updateLiveCompositeKeymap(key_addr, mapped_key);
  }
}

void AutoshiftDygma::pressHeldModifiers(void)
{
  for (auto key_addr : KeyAddr::all())
  {
    /* Only keys already down before this cycle: a modifier going down right
     * now is the one interrupting us by rollover, and it must land AFTER the
     * tap, not on it. */
    if (!Runtime.device().isKeyswitchPressed(key_addr) ||
        !Runtime.device().wasKeyswitchPressed(key_addr) ||
        Runtime.device().isKeyMasked(key_addr))
    {
      continue;
    }

    /* The live composite keymap holds what the key actually resolved to, so a
     * qukey or superkey that settled on a modifier as its hold action is picked
     * up too: both emit that modifier as a toggle-on at the key's own address,
     * which overwrites the cached entry. */
    Key key = Layer.lookup(key_addr);

    if ((key.getFlags() & (SYNTHETIC | RESERVED)) ||
        key.getKeyCode() < HID_KEYBOARD_FIRST_MODIFIER ||
        key.getKeyCode() > HID_KEYBOARD_LAST_MODIFIER)
    {
      continue;
    }

    Runtime.hid().keyboard().pressKey(key, false);
  }
}

void AutoshiftDygma::pressOneShotModifiers(void)
{
  /* OneShot re-injects its active modifiers from its own beforeReportingState(),
   * well after the scan, so none of them is in the report yet. */
  for (uint8_t keycode = HID_KEYBOARD_FIRST_MODIFIER; keycode <= HID_KEYBOARD_LAST_MODIFIER; keycode++)
  {
    Key key(keycode, KEY_FLAGS);

    if (::OneShot.isModifierActive(key))
    {
      Runtime.hid().keyboard().pressKey(key, false);
    }
  }
}

void AutoshiftDygma::emitTap(void)
{
  Key key(pending_kc_, 0);

  /* The report is rebuilt from scratch every cycle, key by key in scan order,
   * and emitTap() runs in the middle of that scan. Any modifier sitting on a
   * later row (Shift, Ctrl, Alt, GUI on the bottom rows, most of all) has not
   * been re-pressed yet, so without this the tap would reach the host bare:
   * Ctrl+C would type a plain 'c'. */
  pressHeldModifiers();
  pressOneShotModifiers();

  /* Press and release fit in a single cycle, as long as the report is pushed
   * out between them: releaseKey() is only honoured for INJECTED events, and
   * without the explicit sendReport() it would strip the keycode again before
   * Runtime_::loop() ever sends it. Same pattern as playMacroKeyswitchEvent()
   * in DynamicMacrosDygma.cpp. */
  handleKeyswitchEvent(key, pending_addr_, IS_PRESSED | INJECTED);
  Runtime.hid().keyboard().sendReport();
  handleKeyswitchEvent(key, pending_addr_, WAS_PRESSED | INJECTED);

  /* We consumed the physical press, so OneShot never saw it and would leave
   * its modifiers and layers armed for the next key. Consume them now rather
   * than at the end of the cycle: on rollover, the interrupting key is still
   * to be processed in this very cycle and must not inherit them.
   *
   * Releasing a one-shot modifier strips its keycode from the report, which
   * takes out the same modifier held physically on a key already scanned in
   * this cycle -- so put those back. */
  if (::OneShot.consumeOnKeyPress(key))
  {
    pressHeldModifiers();
  }

  debug_taps_++;
  AS_LOG("[AS] TAP -> emitting plain kc=0x%02X", pending_kc_);
  state_        = State::IDLE;
  pending_addr_ = KeyAddr();
  pending_kc_   = 0;
}

EventHandlerResult AutoshiftDygma::onFocusEvent(const char *command)
{
  if (::Focus.handleHelp(command, "autoshift.enabled\nautoshift.timeout\nautoshift.state"))
    return EventHandlerResult::OK;

  if (strncmp_P(command, "autoshift.", 10) != 0) return EventHandlerResult::OK;

  if (strcmp_P(command + 10, "enabled") == 0)
  {
    if (::Focus.isEOL())
    {
      ::Focus.send(p_autoshift_config->enabled);
    }
    else
    {
      uint8_t enabled = 0;
      ::Focus.read(enabled);

      cfgmem_enabled_save( (enabled != 0) ? 1 : 0 );
    }
  }

  if (strcmp_P(command + 10, "timeout") == 0)
  {
    if (::Focus.isEOL())
    {
      ::Focus.send(p_autoshift_config->hold_timeout_ms);
    }
    else
    {
      uint16_t hold_timeout_ms = 0;
      ::Focus.read(hold_timeout_ms);

      /* Clamp rather than trust the host: an out-of-range value would either
       * make every key feel stuck or make the hold unreachable. */
      if (hold_timeout_ms < MIN_HOLD_TIMEOUT_MS) hold_timeout_ms = MIN_HOLD_TIMEOUT_MS;
      if (hold_timeout_ms > MAX_HOLD_TIMEOUT_MS) hold_timeout_ms = MAX_HOLD_TIMEOUT_MS;

      cfgmem_hold_timeout_save( hold_timeout_ms );
    }
  }

  /* Read-only diagnostics: enabled, timeout, state, then the counters
   * seen / taps / holds / bypassed / degraded.
   *
   * `seen` is the one that matters first: if it stays at zero after pressing
   * the key, the plugin never received an AUTOSHIFT_* code, so the key is not
   * really assigned in the active keymap -- no amount of plugin debugging will
   * help until that is fixed. */
  if (strcmp_P(command + 10, "state") == 0)
  {
    ::Focus.send(p_autoshift_config != nullptr ? p_autoshift_config->enabled : (uint8_t)0);
    ::Focus.send(p_autoshift_config != nullptr ? p_autoshift_config->hold_timeout_ms : (uint16_t)0);
    ::Focus.send((uint8_t)state_);
    ::Focus.send(debug_seen_);
    ::Focus.send(debug_taps_);
    ::Focus.send(debug_holds_);
    ::Focus.send(debug_bypassed_);
    ::Focus.send(debug_degraded_);
  }

  return EventHandlerResult::EVENT_CONSUMED;
}

void AutoshiftDygma::reserveStorage()
{
  storage_base_ = ::EEPROMSettings.requestAppendedSlice(sizeof(autoshift_config_t));
  Runtime.storage().get(storage_base_, config_);
  p_autoshift_config = &config_;

  /* Sanitize against the 0xFF pattern of erased flash: units upgrading from a
   * firmware without this block read the new tail as all-ones. */
  if( p_autoshift_config->hold_timeout_ms < MIN_HOLD_TIMEOUT_MS ||
      p_autoshift_config->hold_timeout_ms > MAX_HOLD_TIMEOUT_MS )
  {
    cfgmem_hold_timeout_save( DEFAULT_HOLD_TIMEOUT_MS );
  }

  if( p_autoshift_config->enabled > 1 )
  {
    cfgmem_enabled_save( 1 );
  }
}

/****************************************************/
/*                   Config Memory                  */
/****************************************************/

void AutoshiftDygma::cfgmem_hold_timeout_save( uint16_t hold_timeout_ms )
{
  config_.hold_timeout_ms = hold_timeout_ms;
  cfgmem_commit();
}

void AutoshiftDygma::cfgmem_enabled_save( uint8_t enabled )
{
  config_.enabled = enabled;
  cfgmem_commit();
}

void AutoshiftDygma::cfgmem_commit( void )
{
  Runtime.storage().put(storage_base_, config_);
  Runtime.storage().commit();
}

} // namespace plugin
} // namespace kaleidoscope

kaleidoscope::plugin::AutoshiftDygma Autoshift;
