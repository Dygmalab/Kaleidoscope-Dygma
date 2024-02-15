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
}

void Superkey::run()
{
    // Check time being press. If the timer expires, call timeout event.
    // Check if the key is being hold enough time.
    // Check if superKeyState.triggered = true, check sk type and send the corresponding key to the OS.
    // Check if the sk has to be interrupted by any external event.
    if (superKeyState.is_qukey && superKeyState.released){
        timeout();
        disable();
    }
    if (kaleidoscope::Runtime_::hasTimeExpired(superKeyState.timeStamp,time_out_)){
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
    if (kaleidoscope::Runtime_::hasTimeExpired(superKeyState.hold_start,hold_start_) ){
        hold();
    }
    update_timestamp();
}

/************SUPERKEY ACTIONS*************/

void Superkey::tap()
{
    superKeyState.released = false;
    superKeyState.hold_start =  kaleidoscope::Runtime_::millisAtCycleStart();
    update_timestamp();
    ++superKeyState.tap_count;
}

void Superkey::hold()
{
    superKeyState.holded = true;
    superKeyState.released = false;
    if (!superKeyState.triggered){
        send_key();
    }
    superKeyState.triggered = true; //then, if we continue holding the key, we will set ir as pressed, and take the corresponding actions.
}

void Superkey::release()
{
    superKeyState.released = true;
    ++superKeyState.tap_count;
    //Restar timer.
    superKeyState.hold_start =  kaleidoscope::Runtime_::millisAtCycleStart();
}

void Superkey::timeout()
{
    if (!superKeyState.triggered){
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

//TODO: move check_if_sk_interruptable and find_key_type to the ActionDriver. Asi el ActionDriver es el que se encarga de filtrar las teclas y asignar las acciones.
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
    ActionsDriver::action_handler(superKeyState.tap_count, Actions, phisical_key_, keyaddr_);
}
