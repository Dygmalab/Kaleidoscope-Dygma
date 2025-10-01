#include "Superkey.h"
/*Variable declarations*/

/************SUPERKEY CONFIGURATION*************/
void Superkey::init(const Key *act)
{
    set_up_actions(act);
    check_if_sk_qukey();
}

void Superkey::enable(uint8_t modifiers_pressed)
{
    // This is to tell the SK handler to remove the SK from the list of active sk.
    superKeyState.enabled = true;
    superKeyState.cache_modifiers = modifiers_pressed;
}

void Superkey::disable()
{
/*    NRF_LOG_DEBUG("Disabling Superkey %i", index_);
    NRF_LOG_DEBUG("----------------------------------------");*/
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
    superKeyState.is_repeateable = false;
    timeline.remove(this->keyaddr_);
}

void Superkey::run()
{
    // Check time being press. If the timer expires, call timeout event.
    // Check if the key is being hold enough time.
    // Check if superKeyState.triggered = true, check sk type and send the corresponding key to the OS.
    // Check if the sk has to be interrupted by any external event.
    // NRF_LOG_DEBUG("Running Superkey %i", index_);
    if (superKeyState.is_qukey && superKeyState.released)
    {
        timeout();
        disable();
    }
    else if (kaleidoscope::Runtime_::hasTimeExpired(superKeyState.timeStamp, time_out_))
    {
        timeout();
        disable();
    }
}

/************SUPERKEY STATES*************/
void Superkey::key_pressed()
{
    tap();
}

void Superkey::key_released()
{
    release();
}

void Superkey::key_is_pressed()
{
    if (kaleidoscope::Runtime_::hasTimeExpired(superKeyState.hold_start, hold_start_))
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
    superKeyState.minimum_hold = superKeyState.hold_start;
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
    superKeyState.released = true;
    superKeyState.is_repeateable = false; // we stop sending the key to the OS.
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
        //NRF_LOG_DEBUG("Superkey %i is being interrupted by key %i", index_, regular_key.getRaw());
        if (kaleidoscope::Runtime_::hasTimeExpired(superKeyState.minimum_hold, minimum_hold_start_) && !superKeyState.is_qukey)
        {
            superKeyState.tap_count = (uint8_t)Utils::EventType::HOLD; // Force the action tap count to TAP.
        }
        else
        {
            superKeyState.tap_count = (uint8_t)Utils::EventType::TAP; // Force the action tap count to TAP.
        }
        timeout();
        disable();
        result = true;
    }
    return result;
}

void Superkey::set_up_actions(const Key *act)
{
    // Set each action in the superkeys.
    for (int i = 0; i < KEYS_IN_SUPERKEY; ++i)
    {
        Actions[i] = act[i];
    }
}

bool Superkey::is_interruptible()
{
    superKeyState.is_interruptable = ActionsDriver::return_type(superKeyState.tap_count, Actions).key_is_interruptable;
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
    // Check if Actions[0] and Actions[1] are configured (not idle) and Actions[2] to Actions[5] are not configured (idle)
    bool first_two_configured = (Actions[0].getRaw() != IDLE_KEY) && (Actions[1].getRaw() != IDLE_KEY);
    bool rest_idle = (Actions[2].getRaw() == IDLE_KEY) && (Actions[3].getRaw() == IDLE_KEY) && (Actions[4].getRaw() == IDLE_KEY);

    if (first_two_configured && rest_idle)
    {
        superKeyState.is_qukey = true;
    }
    else
    {
        superKeyState.is_qukey = false;
    }
}

void Superkey::keep_sending_hold_key(bool holded)
{
    superKeyState.is_repeateable = holded;
}

//*********************************************************************************************
bool Superkey::is_enable() const
{
    return superKeyState.enabled;
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

    ActionsDriver::action_handler(superKeyState.tap_count, Actions, phisical_key_, keyaddr_);
}
