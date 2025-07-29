#include "Superkey.h"
/*Variable declarations*/

/************SUPERKEY CONFIGURATION*************/
void Superkey::init(const Key *act)
{
    set_up_actions(act);
    check_if_sk_qukey();
}

void Superkey::enable()
{
    // This is to tell the SK handler to remove the SK from the list of active sk.
    superKeyState.enabled = true;
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
    superKeyState.is_repeateable = false;
}

void Superkey::run()
{
    // Check time being press. If the timer expires, call timeout event.
    // Check if the key is being hold enough time.
    // Check if superKeyState.triggered = true, check sk type and send the corresponding key to the OS.
    // Check if the sk has to be interrupted by any external event.
    if (superKeyState.is_qukey && superKeyState.released)
    {
        timeout();
        disable();
    }

    if (kaleidoscope::Runtime_::hasTimeExpired(superKeyState.timeStamp, time_out_))
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
    update_timestamp();
    ++superKeyState.tap_count;
}

void Superkey::hold()
{
    superKeyState.holded = true;
    superKeyState.released = false;
    if (!superKeyState.triggered)
    {
        send_key();
    }
    //! we want to check if the holded key should continue sending the key to the OS. e.g: shift
    superKeyState.triggered = true; // then, if we continue holding the key, we will set it as pressed, and take the corresponding actions.
}

void Superkey::release()
{
    superKeyState.released = true;
    superKeyState.is_repeateable = false; // we stop sending the key to the OS.
    ++superKeyState.tap_count;
    // Restar timer.
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

void Superkey::interrupt()
{
    if (superKeyState.is_interruptable)
    {
        superKeyState.interrupt = true;
    }
}

void Superkey::set_up_actions(const Key *act)
{
    // Set each action in the superkeys.
    for (int i = 0; i < KEYS_IN_SUPERKEY; ++i)
    {
        Actions[i] = act[i];
    }
}

void Superkey::check_if_sk_qukey()
{
    uint8_t idle_actions = 0;
    for (auto Action : Actions)
    {
        if (Action.getRaw() == IDLE_KEY)
        {
            ++idle_actions;
        }
    }
    // If we have three idle actions or in other words, if we only have two actions set in this SK, we know this is a qukey.
    if (idle_actions >= QUKEY_MIN_IDLE_ACTIONS)
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
    ActionsDriver::action_handler(superKeyState.tap_count, Actions, phisical_key_, keyaddr_, superKeyState.is_repeateable);
}
