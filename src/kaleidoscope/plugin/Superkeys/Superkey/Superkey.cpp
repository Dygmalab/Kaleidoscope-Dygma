#include "Superkey.h"
/*Variable declarations*/
static constexpr Superkey::Range ranges[] = {
    {Superkey::LAYER_LOCK_FIRST, Superkey::LAYER_LOCK_LAST},
    {Superkey::LED_BUTTONS_FIRST, Superkey::LED_BUTTONS_LAST},
    {kaleidoscope::ranges::DYNAMIC_MACRO_FIRST, kaleidoscope::ranges::DYNAMIC_MACRO_LAST},
    {Superkey::ALPHA_WITH_MODIFIERS_FIRST, Superkey::ALPHA_WITH_MODIFIERS_LAST},
    {0, Superkey::ALPHA_KEYS},
    {Superkey::LAYER_SHIFT_FIRST, Superkey::LAYER_SHIFT_LAST},
    {23785, 23786} // maybe these are LEDs buttons.
};

static constexpr int numRanges = sizeof(ranges) / sizeof(ranges[0]);

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
    superKeyState.tap_count = 0;
    superKeyState.holded = false;
    superKeyState.triggered = false;
    superKeyState.type = TapType::None;
    superKeyState.interrupt = false;
    superKeyState.start_time = 0;
    superKeyState.hold_start = 0;
    superKeyState.timeStamp = 0;
    superKeyState.pressed = false;
    superKeyState.enabled = false;
}

void Superkey::run()
{
    // Check time being press. If the timer expires, call timeout event.
    // Check if the key is being hold enough time.
    // Check if superKeyState.triggered = true, check sk type and send the corresponding key to the OS.
    // Check if the sk has to be interrupted by any external event.
    if (kaleidoscope::Runtime_::hasTimeExpired(superKeyState.timeStamp,time_out_)){
        NRF_LOG_DEBUG("Releasing key!");

        //sendKey();
        disable();
    } else if (kaleidoscope::Runtime_::hasTimeExpired(superKeyState.hold_start,hold_start_)){
        hold();
    }
}

/************SUPERKEY STATES*************/
void Superkey::key_pressed()
{
    superKeyState.pressed = true;
    superKeyState.tap_count++;
    superKeyState.hold_start =  kaleidoscope::Runtime_::millisAtCycleStart();
    NRF_LOG_DEBUG("superkey with index %i pressed ",index_);
    tap();
}

void Superkey::key_released()
{
    superKeyState.pressed = false;
    superKeyState.tap_count++;
    superKeyState.hold_start = 0;
    NRF_LOG_DEBUG("superkey with index %i released ",index_);
}

void Superkey::key_is_pressed()
{
    update_timestamp();
}

/************SUPERKEY ACTIONS*************/

uint8_t Superkey::tap()
{
    return ++superKeyState.tap_count;
}

uint8_t Superkey::hold()
{
    superKeyState.holded = true;
    superKeyState.is_being_hold = false; //Set it on false in order to treat a hold just one time.
    //sendkey();
    superKeyState.is_being_hold = true; //then, if we continue holding the key, we will set ir as pressed, and take the corresponding actions.
    return true;
}

uint8_t Superkey::release()
{
    superKeyState.holded = false;
    return ++superKeyState.tap_count;
}

void Superkey::timeout()
{
    superKeyState.triggered = true;
}

void Superkey::interrupt()
{
    if (superKeyState.is_interruptable)
    {
        superKeyState.interrupt = true;
    }
}

/***************SUPERKEY TYPE****************/
void Superkey::set_tap_action()
{
    superKeyState.type = TapType::Tap_Once;
}
void Superkey::set_hold_action()
{
    superKeyState.type = TapType::Hold_Once;
}
void Superkey::set_tap_and_hold_action()
{
    superKeyState.type = TapType::Tap_Hold;
}
void Superkey::set_double_tap_action()
{
    superKeyState.type = TapType::Tap_Twice;
}
void Superkey::set_double_tap_hold_action()
{
    superKeyState.type = TapType::Tap_Twice_Hold;
}

void Superkey::set_up_actions(const Key *act)
{
    // Set each action in the superkeys.
    for (int i = 0; i < KEYS_IN_SUPERKEY; ++i)
    {
        superKeyState.Actions[i] = act[i];
    }
}

void Superkey::check_if_sk_qukey()
{
    uint8_t idle_actions = 0;
    for (auto Action : superKeyState.Actions)
    {
        if (Action.getRaw() == 1)
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

void Superkey::check_if_sk_interruptable(const Key &Action)
{
    auto ranges_t = static_cast<KeyRanges>(find_key_type(Action.getRaw()));
    switch (ranges_t)
    {
        case KeyRanges::LAYER_LOCK:
        case KeyRanges::DYNAMIC_MACRO:
        case KeyRanges::LAYER_SHIFT:
        case KeyRanges::ALPHA_WITH_MODIFIERS:
            superKeyState.is_interruptable = false;
            break;
        default:
            superKeyState.is_interruptable = true;
            break;
    }

    if (superKeyState.is_interruptable)
    {
        NRF_LOG_DEBUG("superkey with index %i is interruptable ", index_);
    }
    else
    {
        NRF_LOG_DEBUG("superkey with index %i is NOT interruptable ", index_);
    }
}

uint16_t Superkey::find_key_type(uint16_t value)
{
    int start = 0;
    int end = numRanges - 1;

    while (start <= end)
    {
        int mid = start + (end - start) / 2;

        if (value >= ranges[mid].start && value <= ranges[mid].end)
        {
            return mid; // Found the range
        }
        else if (value < ranges[mid].start)
        {
            end = mid - 1; // Search in the left half
        }
        else
        {
            start = mid + 1; // Search in the right half
        }
    }
    return -1;
}

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
