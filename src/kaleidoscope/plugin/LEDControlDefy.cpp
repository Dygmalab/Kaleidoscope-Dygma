/* Kaleidoscope-LEDControl - LED control plugin for Kaleidoscope
 * Copyright (C) 2017-2020  Keyboard.io, Inc.
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

#include "Colormap-Defy.h"
#include "EEPROM-Settings.h"
#include "Kaleidoscope-FocusSerial.h"
#include "Kaleidoscope-LEDControl.h"
#include "kaleidoscope/keyswitch_state.h"
#include "kaleidoscope_internal/LEDModeManager.h"

using namespace kaleidoscope::internal; // NOLINT(build/namespaces)

namespace kaleidoscope
{
namespace plugin
{
uint16_t LEDControl::settings_base_ = 0;
static constexpr uint8_t uninitialized_mode_id = 255;
uint8_t LEDControl::fade_effect = 0;
uint8_t LEDControl::mode_id = uninitialized_mode_id;
uint8_t LEDControl::num_led_modes_ = LEDModeManager::numLEDModes();
LEDMode *LEDControl::cur_led_mode_ = nullptr;
uint8_t LEDControl::syncDelay = 32;
uint16_t LEDControl::syncTimer = 0;
bool LEDControl::enabled_ = true;
Key LEDControl::pending_next_prev_key_ = Key_NoKey;
uint8_t user_effects = 3; // Battery status, advertising mode.
bool LEDControl::force = false;

LEDControl::LEDControl(void)
{
}

void LEDControl::set_force_mode(bool status)
{
    force = status;
}

void LEDControl::next_mode(void)
{
    mode_id++;

    if (mode_id >= num_led_modes_ - user_effects)
    {
        return set_mode(0);
    }

    return set_mode(mode_id);
}

void LEDControl::prev_mode(void)
{
    if (mode_id == 0)
    {
        // wrap around
        mode_id = num_led_modes_ - user_effects;
    }
    else
    {
        mode_id--;
    }

    return set_mode(mode_id);
}

void LEDControl::set_mode(uint8_t mode_)
{
    if (mode_ >= num_led_modes_) return;

    mode_id = mode_;

    // Cache the LED mode
    //
    cur_led_mode_ = LEDModeManager::getLEDMode(mode_id);

    refreshAll();

    kaleidoscope::Hooks::onLEDModeChange();
}

void LEDControl::activate(LEDModeInterface *plugin)
{
    for (uint8_t i = 0; i < num_led_modes_; i++)
    {

        led_mode_management::LEDModeFactory fac;

        LEDModeManager::retreiveLEDModeFactoryFromPROGMEM(i, fac);

        if (fac.isAssociatedWithPlugin(plugin))
        {
            set_mode(i);
            return;
        }
    }
}

void LEDControl::set_all_leds_to(uint8_t r, uint8_t g, uint8_t b)
{
    if (!Runtime.has_leds) return;

    cRGB color;
    color.r = r;
    color.g = g;
    color.b = b;
    set_all_leds_to(color);
}

void LEDControl::set_all_leds_to(cRGB color)
{
    for (auto led_index : Runtime.device().LEDs().all())
    {
        setCrgbAt(led_index.offset(), color);
    }
}

void LEDControl::setCrgbAt(uint8_t led_index, cRGB crgb)
{
    Runtime.device().setCrgbAt(led_index, crgb);
}

void LEDControl::setCrgbAt(KeyAddr key_addr, cRGB color)
{
    Runtime.device().setCrgbAt(key_addr, color);
}

cRGB LEDControl::getCrgbAt(uint8_t led_index)
{
    return Runtime.device().getCrgbAt(led_index);
}
cRGB LEDControl::getCrgbAt(KeyAddr key_addr)
{
    return Runtime.device().getCrgbAt(Runtime.device().getLedIndex(key_addr));
}

void LEDControl::syncLeds(void)
{
    if (force) return;
    if (!enabled_) return;

    Runtime.device().syncLeds();
}

kaleidoscope::EventHandlerResult LEDControl::onSetup()
{
    settings_base_ = kaleidoscope::plugin::EEPROMSettings::requestSlice(sizeof(fade_effect));

    Runtime.storage().get(settings_base_, fade_effect);
    // For now lest think that if one block is invalid restart everything
    if (fade_effect == 0xFF)
    {
        fade_effect = 0;
        Runtime.storage().put(settings_base_, fade_effect);
        Runtime.storage().commit();
    }
    Runtime.storage().get(settings_base_, fade_effect);
    ::ColormapEffectDefy.setFadein(fade_effect);
    set_all_leds_to({0, 0, 0});

    LEDModeManager::setupPersistentLEDModes();

    if (mode_id == uninitialized_mode_id)
    {
        set_mode(0);
    }

    return EventHandlerResult::OK;
}

void LEDControl::disable()
{
    set_all_leds_to(CRGB(0, 0, 0));
    enabled_ = false;
    Runtime.device().syncLeds();
}

void LEDControl::enable()
{
    enabled_ = true;
    refreshAll();
    Runtime.device().syncLeds();
}

kaleidoscope::EventHandlerResult LEDControl::onKeyswitchEvent(Key &mappedKey, KeyAddr key_addr, uint8_t keyState)
{
    if (mappedKey.getFlags() != (SYNTHETIC | IS_INTERNAL | LED_TOGGLE)) return kaleidoscope::EventHandlerResult::OK;

    if (keyToggledOn(keyState))
    {
        if (mappedKey == Key_LEDEffectNext || mappedKey == Key_LEDEffectPrevious)
        {
            // Handling of these keys is delayed into `beforeReportingState`
            // so that we can incorporate the shift modifier state.
            pending_next_prev_key_ = mappedKey;
        }
        else if (mappedKey == Key_LEDToggle)
        {
            if (enabled_)
                disable();
            else
                enable();
        }
    }

    return kaleidoscope::EventHandlerResult::EVENT_CONSUMED;
}

kaleidoscope::EventHandlerResult LEDControl::beforeReportingState(void)
{
    if (!enabled_) return kaleidoscope::EventHandlerResult::OK;

    if (pending_next_prev_key_ != Key_NoKey)
    {
        bool is_shifted =
            kaleidoscope::Runtime.hid().keyboard().isModifierKeyActive(Key_LeftShift) || kaleidoscope::Runtime.hid().keyboard().isModifierKeyActive(Key_RightShift);

        if ((pending_next_prev_key_ == Key_LEDEffectNext && !is_shifted) || (pending_next_prev_key_ == Key_LEDEffectPrevious && is_shifted))
        {
            next_mode();
        }
        else
        {
            prev_mode();
        }
        pending_next_prev_key_ = Key_NoKey;
    }

    if (Runtime.hasTimeExpired(syncTimer, syncDelay))
    {
        syncLeds();
        syncTimer += syncDelay;
        update();
    }

    return kaleidoscope::EventHandlerResult::OK;
}
void LEDControl::refreshAll()
{
    if (!Runtime.has_leds) return;

    if (mode_id < num_led_modes_ - user_effects)
    {
        if (!enabled_) return;
    }

    set_all_leds_to({0, 0, 0});

    if (cur_led_mode_ != nullptr) cur_led_mode_->onActivate();
}

EventHandlerResult FocusLEDCommand::onFocusEvent(const char *command)
{
    enum
    {
        SETALL,
        MODE,
        AT,
        THEME,
        BRIGHTNESS_WIRED,
        BRIGHTNESS_UG_WIRED,
        BRIGHTNESS_WIRELESS,
        BRIGHTNESS_UG_WIRELESS,
        FADE_UG
    } subCommand;

    if (!Runtime.has_leds) return EventHandlerResult::OK;

    if (::Focus.handleHelp(command, "led.at\n"
                                    "led.setAll\n"
                                    "led.mode\n"
                                    "led.brightness\n"
                                    "led.brightnessUG\n"
                                    "led.brightness.wireless\n"
                                    "led.brightnessUG.wireless\n"
                                    "led.fade\n"
                                    "led.theme"))
        return EventHandlerResult::OK;

    if (strncmp(command, "led.", 4) != 0) return EventHandlerResult::OK;
    if (strcmp(command + 4, "at") == 0)
        subCommand = AT;
    else if (strcmp(command + 4, "setAll") == 0)
        subCommand = SETALL;
    else if (strcmp(command + 4, "mode") == 0)
        subCommand = MODE;
    else if (strcmp(command + 4, "theme") == 0)
        subCommand = THEME;
    else if (strcmp(command + 4, "brightness") == 0)
        subCommand = BRIGHTNESS_WIRED;
    else if (strcmp(command + 4, "brightnessUG") == 0)
        subCommand = BRIGHTNESS_UG_WIRED;
    else if (strcmp(command + 4, "brightness.wireless") == 0)
        subCommand = BRIGHTNESS_WIRELESS;
    else if (strcmp(command + 4, "brightnessUG.wireless") == 0)
        subCommand = BRIGHTNESS_UG_WIRELESS;
    else if (strcmp(command + 4, "fade") == 0)
        subCommand = FADE_UG;
    else
        return EventHandlerResult::OK;

    switch (subCommand)
    {
        case AT:
        {
            uint8_t idx;

            ::Focus.read(idx);

            if (::Focus.isEOL())
            {
                cRGB c = ::LEDControl.getCrgbAt(idx);

                ::Focus.send(c);
            }
            else
            {
                cRGB c;

                ::Focus.read(c);

                ::LEDControl.setCrgbAt(idx, c);
            }
            break;
        }
        case BRIGHTNESS_WIRED:
        {
            if (::Focus.isEOL())
            {
                ::Focus.send(::LEDControl.getBrightness());
            }
            else
            {
                uint8_t brightness;

                ::Focus.read(brightness);
                ::LEDControl.setBrightness(brightness);
            }
            break;
        }
        case BRIGHTNESS_UG_WIRED:
        {
            if (::Focus.isEOL())
            {
                ::Focus.send(::LEDControl.getBrightnessUG());
            }
            else
            {
                uint8_t brightness;

                ::Focus.read(brightness);
                ::LEDControl.setBrightnessUG(brightness);
            }
            break;
        }
        case BRIGHTNESS_WIRELESS:
        {
            if (::Focus.isEOL())
            {
                ::Focus.send(::LEDControl.getBrightnessWireless());
            }
            else
            {
                uint8_t brightness;

                ::Focus.read(brightness);
                ::LEDControl.setBrightnessWireless(brightness);
            }
            break;
        }
        case BRIGHTNESS_UG_WIRELESS:
        {
            if (::Focus.isEOL())
            {
                ::Focus.send(::LEDControl.getBrightnessUGWireless());
            }
            else
            {
                uint8_t brightness;

                ::Focus.read(brightness);
                ::LEDControl.setBrightnessUGWireless(brightness);
            }
            break;
        }
        case FADE_UG:
        {
            if (::Focus.isEOL())
            {
                ::Focus.send(::LEDControl.FadeUGIsActivated());
            }
            else
            {
                uint8_t fade_ug_effect;

                ::Focus.read(fade_ug_effect);
                ::LEDControl.activateFadeUG(fade_ug_effect);
                ::ColormapEffectDefy.setFadein(fade_ug_effect);
                Runtime.storage().put(LEDControl::settings_base_, fade_ug_effect);
                Runtime.storage().commit();
            }
        }
        case SETALL:
        {
            if (!::Focus.isEOL())
            {
                cRGB c;
                ::Focus.read(c);
                ::LEDControl.set_all_leds_to(c);
            }
            break;
        }
        case MODE:
        {
            char peek = ::Focus.peek();
            if (peek == '\n')
            {
                ::Focus.send(::LEDControl.get_mode_index());
            }
            else if (peek == 'n')
            {
                ::LEDControl.next_mode();
            }
            else if (peek == 'p')
            {
                ::LEDControl.prev_mode();
            }
            else
            {
                uint8_t mode_id;

                ::Focus.read(mode_id);
                ::LEDControl.set_mode(mode_id);
            }
            break;
        }
        case THEME:
        {
            if (::Focus.isEOL())
            {
                for (auto led_index : Runtime.device().LEDs().all())
                {
                    cRGB c = ::LEDControl.getCrgbAt(led_index.offset());

                    ::Focus.send(c);
                }
                break;
            }

            for (auto led_index : Runtime.device().LEDs().all())
            {
                if (::Focus.isEOL())
                {
                    break;
                }

                cRGB color;

                ::Focus.read(color);

                ::LEDControl.setCrgbAt(led_index.offset(), color);
            }
            break;
        }
    }

    return EventHandlerResult::EVENT_CONSUMED;
}

} // namespace plugin
} // namespace kaleidoscope

kaleidoscope::plugin::LEDControl LEDControl;
kaleidoscope::plugin::FocusLEDCommand FocusLEDCommand;

#endif