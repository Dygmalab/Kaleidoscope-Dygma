/* Superkey - Superkey support for Kaleidoscope.
 * Copyright (C) 2025 Dygma Lab S.L.
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
#include "Superkeys/Actions/ActionsDriver.h"
#include "Superkey.h"


Superkey::Superkey()
    : phisical_key_{}
    , keyaddr_{}
    , index_{0}
    , shared_config_{nullptr}
{
    // Initialize bitfields explicitly
    superKeyState.pressed = 0;
    superKeyState.triggered = 0;
    superKeyState.holded = 0;
    superKeyState.released = 0;
    superKeyState.interrupt = 0;
    superKeyState.enabled = 0;
    superKeyState.is_qukey = 0;
    superKeyState.is_interruptable = 0;
}

/************SUPERKEY CONFIGURATION*************/
void Superkey::init(const superkey_config_t *p_superkey_config)
{
    set_up_superkey_config(p_superkey_config);
    check_if_sk_qukey();
}

void Superkey::enable(uint8_t modifiers_pressed)
{
    // This is to tell the SK handler to remove the SK from the list of active sk.
    superKeyState.enabled = true;
    superKeyState.pressed = true;  // Mark as pressed so run() can process it
    superKeyState.cache_modifiers = modifiers_pressed;
}

void Superkey::disable()
{
    superKeyState.enabled = false;
    superKeyState.tap_count = 0;
    superKeyState.holded = false;
    superKeyState.triggered = false;
    superKeyState.type = Utils::TapType::None;
    superKeyState.interrupt = false;
    superKeyState.start_time = 0;
    superKeyState.hold_start = 0;
    superKeyState.timeStamp = 0;
    superKeyState.pressed = false;
    superKeyState.released = false;
    timeline.remove(this->keyaddr_);
}

void Superkey::run()
{
    // Check time being press. If the timer expires, call timeout event.
    // Check if the key is being hold enough time.
    // Check if superKeyState.triggered = true, check sk type and send the corresponding key to the OS.
    // Check if the sk has to be interrupted by any external event.
    // NRF_LOG_DEBUG("Running Superkey %i", index_);
    if (superKeyState.is_qukey)
    {
        if (superKeyState.released)
        {
            // Preserve ordering: if there is an earlier superkey still pending,
            // defer qukey finalization until it completes.
            bool has_prev = timeline.has_previous_superkey_pending(this->keyaddr_);
            if (!has_prev)
            {
                timeout();
                disable();
            }
            else
            {
                // Wait for previous superkeys to finalize first
            }
        }
        // For qukeys, do not apply the generic timeout path while release ordering is constrained.
    }
    else
    {
        // Handle interrupted state for normal superkeys
        if (superKeyState.released && superKeyState.interrupt)
        {
            // Was interrupted by another key: preserve ordering
            bool has_prev = timeline.has_previous_superkey_pending(this->keyaddr_);
            if (!has_prev)
            {
                timeout();
                disable();
            }
            else
            {
                // Wait for previous superkeys to finalize first
            }
        }
        else if (superKeyState.pressed)
        {
            // Key is still pressed (not interrupted, not released)
            // Check if it should transition to hold state
            key_is_pressed();
        }
        
        // Check timeout for both pressed and released states
        // This allows detecting multiple taps after release
        if (shared_config_ && kaleidoscope::Runtime_::hasTimeExpired(superKeyState.timeStamp, shared_config_->time_out_))
        {
            timeout();
            disable();
        }
    }
}

/************SUPERKEY STATES*************/
void Superkey::key_pressed()
{
    tap();
}

void Superkey::key_released()
{
    // Capture whether this cycle was a hold before release() resets flags
    bool was_hold = superKeyState.holded;
    release();
    
    // For qukeys: if there's no earlier superkey pending, finalize immediately
    // on release to ensure the instance is disabled before a rapid next press.
    if (superKeyState.is_qukey)
    {
        bool has_previous = timeline.has_previous_superkey_pending(this->keyaddr_);
        if (!has_previous)
        {
            timeout();
            disable();
            return;
        }
        else
        {
            // Defer finalization to preserve timeline order
        }
    }
    // If it was a hold (already triggered) and not a qukey, we can safely
    // finalize immediately on release to avoid blocking the next press.
    if (was_hold && !superKeyState.is_qukey)
    {
        timeout();
        disable();
    }
    else
    {
        // Will finalize in run() after timeout
    }
}

void Superkey::key_is_pressed()
{
    if (shared_config_ && kaleidoscope::Runtime_::hasTimeExpired(superKeyState.hold_start, shared_config_->hold_start_))
    {
        hold();
    }
    update_timestamp();
}

/************SUPERKEY ACTIONS*************/

void Superkey::tap()
{
    superKeyState.released = false;
    superKeyState.hold_start = kaleidoscope::Runtime_::millisAtCycleStart();
    update_timestamp();
    ++superKeyState.tap_count;
}
//TODO: if I press shift + a superkey and release shift and then the superkey, it does not come out shifted.
void Superkey::hold()
{
    superKeyState.holded = true;
    superKeyState.released = false;
    if (!superKeyState.triggered)
    {
        send_key();
    }
    //! we want to check if the held key should continue sending the key to the OS. e.g: shift
    superKeyState.triggered = true; // then, if we continue holding the key, we will set it as pressed, and take the corresponding actions.
}

void Superkey::release()
{
    superKeyState.pressed = false;  // Physical key is no longer pressed
    superKeyState.released = true;
    superKeyState.holded = false;
    ++superKeyState.tap_count;
    // Restart timer.
    superKeyState.hold_start = kaleidoscope::Runtime_::millisAtCycleStart();
}

void Superkey::timeout()
{
    if (!superKeyState.triggered)
    {
        superKeyState.triggered = true;
        send_key();
    }
}

bool Superkey::interrupt(Key &regular_key, const KeyAddr &keyaddr_)
{
    bool result = false;

    // If the regular key press is a modifier key, we send it as a modifier to the OS so the next superkeys can use it.
    if (ActionsDriver::isOnlyModifier(regular_key))
    {
        ActionsDriver::send_modifier(regular_key, keyaddr_);
        return false;
    }

    if (!superKeyState.holded && ActionsDriver::key_can_interrupt(regular_key))
    {
        // Decide whether this interruption should finalize as HOLD, TAP, or DOUBLE TAP.
        // If we've already completed a double tap (tap_count >= 2), preserve it.
        // Otherwise, collapse to TAP.
        if (static_cast<uint8_t>(superKeyState.tap_count) < static_cast<uint8_t>(Utils::TapType::Tap_Once))
        {
            superKeyState.tap_count = (uint8_t)Utils::EventType::TAP;
        }
        // else: keep current tap_count (double tap or more)
        
        // Mark as interrupted and released to trigger finalization in run() cycle
        // This preserves timeline order instead of sending immediately
        superKeyState.interrupt = true;
        superKeyState.released = true;
        result = true;
    }
    return result;
}

void Superkey::set_up_superkey_config(const superkey_config_t * p_superkey_config)
{
    // Actions are now stored externally, just assign the pointer
    superkey_config_ = p_superkey_config;
}

bool Superkey::is_interruptible()
{
    if (!superkey_config_) return false;
    superKeyState.is_interruptable = ActionsDriver::return_type(superKeyState.tap_count, superkey_config_).key_is_interruptable;
    return superKeyState.is_interruptable;
}

Key Superkey::get_phisical_key() const
{
    return phisical_key_;
}

KeyAddr Superkey::get_keyAddr() const
{
    return keyaddr_;
}

void Superkey::check_if_sk_qukey()
{
    if (!superkey_config_) {
        superKeyState.is_qukey = false;
        return;
    }
    
    // Check if first two actions are configured (not idle) and rest are not configured (idle)
    bool first_two_configured = (getActionRaw(&superkey_config_->actions[0]) != IDLE_KEY) && (getActionRaw(&superkey_config_->actions[1]) != IDLE_KEY);
    bool rest_idle = (getActionRaw(&superkey_config_->actions[2]) == IDLE_KEY) &&
                     (getActionRaw(&superkey_config_->actions[3]) == IDLE_KEY) &&
                     (getActionRaw(&superkey_config_->actions[4]) == IDLE_KEY);

    if (first_two_configured && rest_idle)
    {
        superKeyState.is_qukey = true;
    }
    else
    {
        superKeyState.is_qukey = false;
    }
}


//*********************************************************************************************
bool Superkey::is_enable() const
{
    return superKeyState.enabled;
}

bool Superkey::is_holded() const
{
    return superKeyState.holded;
}

bool Superkey::is_triggered() const
{
    return superKeyState.triggered;
}

void Superkey::init_timer()
{
    superKeyState.start_time = kaleidoscope::Runtime_::millisAtCycleStart();
    update_timestamp();
}

void Superkey::update_timestamp()
{
    superKeyState.timeStamp = kaleidoscope::Runtime_::millisAtCycleStart();
}

uint16_t Superkey::get_index() const
{
    return index_;
}

bool Superkey::is_qukey() const
{
    return superKeyState.is_qukey;
}

void Superkey::set_key_and_keyAddr(Key key, KeyAddr keyAddr)
{
    phisical_key_ = key;
    keyaddr_ = keyAddr;
}

void Superkey::send_key() const
{
    // Send all cached modifiers
    if (superKeyState.cache_modifiers != 0)
    {
        ActionsDriver::send_modifiers_from_flags(superKeyState.cache_modifiers, keyaddr_);
    }

    if (superkey_config_) {
        ActionsDriver::action_handler(superKeyState.tap_count, superkey_config_, phisical_key_, keyaddr_);
    }
}

uint16_t Superkey::getActionRaw( const action_config_t * p_action_config )
{
    return (uint16_t)(
             ((uint16_t)p_action_config->flags << 8)
             + (uint16_t)p_action_config->keyCode );
}

void Superkey::setActionRaw( action_config_t * p_action_config, uint16_t raw )
{
    p_action_config->flags  = (uint8_t)(raw >> 8);
    p_action_config->keyCode = (uint8_t)(raw & 0x00FF);
}

Key Superkey::getActionAsKey( const action_config_t * p_action_config )
{
    return Key( p_action_config->keyCode, p_action_config->flags );
}

Key Superkey::getActionAsKey( const superkey_config_t * p_superkey, uint16_t action_id )
{
    return getActionAsKey( &p_superkey->actions[action_id] );
}
