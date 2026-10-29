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

#include "CapsWordDygma.h"
#include "Kaleidoscope-EEPROM-Settings.h"

#include "Kaleidoscope-FocusSerial.h"
#include "kaleidoscope/key_events.h"
#include "kaleidoscope/keyswitch_state.h"

/* ---------------------------------------------------------------------------
 * TEMPORARY debug logging. See the matching block in AutoshiftDygma.cpp for
 * why the include is guarded and where the output actually goes.
 * ------------------------------------------------------------------------ */
#define CAPSWORD_DEBUG_LOG 0

#if CAPSWORD_DEBUG_LOG && defined(NRF52_ARCH)
  #define NRF_LOG_MODULE_NAME CAPSWORD
  #define NRF_LOG_LEVEL       4
  #include "nrf_log.h"
  NRF_LOG_MODULE_REGISTER();

  #define CW_LOG(...)  NRF_LOG_INFO(__VA_ARGS__)
#else
  #define CW_LOG(...)  do {} while (0)
#endif

namespace kaleidoscope {
namespace plugin {

/* HID usage ranges we care about. */
static constexpr uint8_t HID_ALPHA_FIRST     = 0x04;  /* 'a' */
static constexpr uint8_t HID_ALPHA_LAST      = 0x1D;  /* 'z' */
static constexpr uint8_t HID_DIGIT_FIRST     = 0x1E;  /* '1' */
static constexpr uint8_t HID_DIGIT_LAST      = 0x27;  /* '0' */
static constexpr uint8_t HID_BACKSPACE       = 0x2A;
static constexpr uint8_t HID_MINUS           = 0x2D;  /* '-' / '_' */
static constexpr uint8_t HID_MODIFIER_FIRST  = 0xE0;
static constexpr uint8_t HID_MODIFIER_LAST   = 0xE7;

/****************************************************/
/*                  CapsWord trigger                */
/****************************************************/

EventHandlerResult CapsWordTriggerDygma::onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state)
{
  if (key_state & INJECTED)
  {
    return EventHandlerResult::OK;
  }

  if (mapped_key.getRaw() != ranges::CAPS_WORD)
  {
    return EventHandlerResult::OK;
  }

  if (keyToggledOn(key_state))
  {
    CW_LOG("[CW] trigger down: enabled=%d active=%d timeout=%d",
           p_capsword_config != nullptr ? p_capsword_config->enabled : 0xFF,
           active_,
           p_capsword_config != nullptr ? p_capsword_config->idle_timeout_ms : 0xFFFF);
    if (p_capsword_config != nullptr && p_capsword_config->enabled != 0)
    {
      active_ = !active_;
      noteActivity();
      CW_LOG("[CW] toggled -> active=%d", active_);
    }
    else
    {
      active_ = false;
    }
  }

  /* Consume every state of the trigger key so the raw range code never leaks
   * further down the chain. */
  return EventHandlerResult::EVENT_CONSUMED;
}

EventHandlerResult CapsWordTriggerDygma::beforeReportingState()
{
  if (!active_ || p_capsword_config == nullptr)
  {
    return EventHandlerResult::OK;
  }

  uint16_t idle_timeout_ms = p_capsword_config->idle_timeout_ms;

  if (idle_timeout_ms != 0 &&
      Runtime.hasTimeExpired(last_activity_ms_, (uint32_t)idle_timeout_ms))
  {
    active_ = false;
    CW_LOG("[CW] idle timeout expired -> off");
  }

  return EventHandlerResult::OK;
}

uint8_t CapsWordTriggerDygma::continuationFlags(void) const
{
  return (p_capsword_config != nullptr) ? p_capsword_config->continuation_flags
                                        : DEFAULT_CONTINUATION;
}

EventHandlerResult CapsWordTriggerDygma::onFocusEvent(const char *command)
{
  if (::Focus.handleHelp(command, "capsword.enabled\ncapsword.timeout\ncapsword.continuation\ncapsword.state"))
    return EventHandlerResult::OK;

  if (strncmp_P(command, "capsword.", 9) != 0) return EventHandlerResult::OK;

  if (strcmp_P(command + 9, "enabled") == 0)
  {
    if (::Focus.isEOL())
    {
      ::Focus.send(p_capsword_config->enabled);
    }
    else
    {
      uint8_t enabled = 0;
      ::Focus.read(enabled);

      cfgmem_enabled_save( (enabled != 0) ? 1 : 0 );

      if (enabled == 0)
      {
        active_ = false;
      }
    }
  }

  if (strcmp_P(command + 9, "timeout") == 0)
  {
    if (::Focus.isEOL())
    {
      ::Focus.send(p_capsword_config->idle_timeout_ms);
    }
    else
    {
      uint16_t idle_timeout_ms = 0;
      ::Focus.read(idle_timeout_ms);

      /* 0 means "never time out" and is legal; clamp the upper end only. */
      if (idle_timeout_ms > MAX_IDLE_TIMEOUT_MS) idle_timeout_ms = MAX_IDLE_TIMEOUT_MS;

      cfgmem_idle_timeout_save( idle_timeout_ms );
    }
  }

  if (strcmp_P(command + 9, "continuation") == 0)
  {
    if (::Focus.isEOL())
    {
      ::Focus.send(p_capsword_config->continuation_flags);
    }
    else
    {
      uint8_t continuation_flags = 0;
      ::Focus.read(continuation_flags);

      cfgmem_continuation_save( continuation_flags & DEFAULT_CONTINUATION );
    }
  }

  /* Read-only diagnostics: enabled, active, timeout, continuation flags.
   *
   * If `enabled` reads back as 1 but pressing the key never flips `active`,
   * the trigger is not receiving the CAPS_WORD code -- which points at the
   * keymap rather than at the plugin. */
  if (strcmp_P(command + 9, "state") == 0)
  {
    ::Focus.send(p_capsword_config != nullptr ? p_capsword_config->enabled : (uint8_t)0);
    ::Focus.send((uint8_t)(active_ ? 1 : 0));
    ::Focus.send(p_capsword_config != nullptr ? p_capsword_config->idle_timeout_ms : (uint16_t)0);
    ::Focus.send(continuationFlags());
  }

  return EventHandlerResult::EVENT_CONSUMED;
}

void CapsWordTriggerDygma::reserveStorage()
{
  storage_base_ = ::EEPROMSettings.requestAppendedSlice(sizeof(capsword_config_t));
  Runtime.storage().get(storage_base_, config_);
  p_capsword_config = &config_;

  /* Sanitize against the 0xFF pattern of erased flash: units upgrading from a
   * firmware without this block read the new tail as all-ones. */
  if( p_capsword_config->idle_timeout_ms > MAX_IDLE_TIMEOUT_MS )
  {
    cfgmem_idle_timeout_save( DEFAULT_IDLE_TIMEOUT_MS );
  }

  if( p_capsword_config->enabled > 1 )
  {
    cfgmem_enabled_save( 1 );
  }

  if( ( p_capsword_config->continuation_flags & ~DEFAULT_CONTINUATION ) != 0 )
  {
    cfgmem_continuation_save( DEFAULT_CONTINUATION );
  }
}

/****************************************************/
/*                   Config Memory                  */
/****************************************************/

void CapsWordTriggerDygma::cfgmem_idle_timeout_save( uint16_t idle_timeout_ms )
{
  config_.idle_timeout_ms = idle_timeout_ms;
  cfgmem_commit();
}

void CapsWordTriggerDygma::cfgmem_enabled_save( uint8_t enabled )
{
  config_.enabled = enabled;
  cfgmem_commit();
}

void CapsWordTriggerDygma::cfgmem_continuation_save( uint8_t continuation_flags )
{
  config_.continuation_flags = continuation_flags;
  cfgmem_commit();
}

void CapsWordTriggerDygma::cfgmem_commit( void )
{
  Runtime.storage().put(storage_base_, config_);
  Runtime.storage().commit();
}

/****************************************************/
/*                   CapsWord shift                 */
/****************************************************/

EventHandlerResult CapsWordShiftDygma::onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state)
{
  if (!CapsWordTrigger.isActive())
  {
    return EventHandlerResult::OK;
  }

  /* Layers, macros, LED and mouse codes are not characters: they neither get
   * shifted nor end the word. Note that keys consumed further up the chain
   * (DynamicMacros, LEDControl, OverlayKey, OneShot) never reach us at all, so
   * they cannot end the word either -- accepted behaviour. */
  if (mapped_key.getFlags() & (SYNTHETIC | RESERVED))
  {
    return EventHandlerResult::OK;
  }

  uint8_t keycode = mapped_key.getKeyCode();

  /* Every operation below is idempotent, which is why this plugin can sit at
   * the end of the chain and tolerate the timeline's double delivery without
   * any (KeyAddr, cycle) de-duplication. */
  if (keycode >= HID_ALPHA_FIRST && keycode <= HID_ALPHA_LAST)
  {
    if (keyToggledOn(key_state))
    {
      CapsWordTrigger.noteActivity();
    }

    /* Applied on holds as well: the flag has to be present on the toggle-on
     * that latches modifier_flag_mask, and on every subsequent held event so
     * requestModifiers() keeps re-arming the shift. */
    if (keyIsPressed(key_state))
    {
      if (keyToggledOn(key_state))
      {
        CW_LOG("[CW] shifting letter kc=0x%02X", keycode);
      }
      mapped_key.setFlags(mapped_key.getFlags() | SHIFT_HELD);
    }

    return EventHandlerResult::OK;
  }

  if (!keyToggledOn(key_state))
  {
    return EventHandlerResult::OK;
  }

  /* Pure modifiers are neutral: they neither end the word nor refresh the
   * idle timer. */
  if (keycode >= HID_MODIFIER_FIRST && keycode <= HID_MODIFIER_LAST)
  {
    return EventHandlerResult::OK;
  }

  uint8_t continuation = CapsWordTrigger.continuationFlags();
  bool    continues    = false;

  if (keycode >= HID_DIGIT_FIRST && keycode <= HID_DIGIT_LAST)
  {
    continues = (continuation & CapsWordTriggerDygma::CONTINUE_DIGITS) != 0;
  }
  else if (keycode == HID_MINUS)
  {
    continues = (continuation & CapsWordTriggerDygma::CONTINUE_DASH) != 0;
  }
  else if (keycode == HID_BACKSPACE)
  {
    continues = (continuation & CapsWordTriggerDygma::CONTINUE_BACKSPACE) != 0;
  }

  if (continues)
  {
    CapsWordTrigger.noteActivity();
  }
  else
  {
    /* Space, enter, tab, escape, punctuation, everything else: word over. */
    CW_LOG("[CW] word ended by kc=0x%02X", keycode);
    CapsWordTrigger.deactivate();
  }

  return EventHandlerResult::OK;
}

} // namespace plugin
} // namespace kaleidoscope

kaleidoscope::plugin::CapsWordTriggerDygma CapsWordTrigger;
kaleidoscope::plugin::CapsWordShiftDygma   CapsWordShift;
