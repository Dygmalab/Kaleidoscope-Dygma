#include "Superkey.h"
/*Variable declarations*/


/************SUPERKEY CONFIGURATION*************/
void Superkey::init()
{
}

void Superkey::config()
{
}

void Superkey::enable()
{
    //This is to tell the SK handler to remove the SK from the list of active sk.
    superKeyState.remove_from_queue = false;
}

void Superkey::disable()
{
    superKeyState.tap_count = 0;
    superKeyState.holded = false;
    superKeyState.triggered = false;
    superKeyState.type = SuperType::None;
    superKeyState.interrupt = false;
    superKeyState.start_time = 0;
    superKeyState.pressed = false;
    superKeyState.remove_from_queue = true;
}

void Superkey::run()
{
    //Check time being press. If the timer expires, call timeout event.
    //Check if the key is being hold enough time.
    //Check if superKeyState.triggered = true, check sk type and send the corresponding key to the OS.
    //Check if the sk has to be interrupted by any external event.
}

/************SUPERKEY STATES*************/
void Superkey::key_pressed()
{
    superKeyState.pressed = true;
    ActionsDriver::event_handler(this,ActionsDriver::EventType::TAP, tap);
}

void Superkey::key_released()
{
    ActionsDriver::event_handler(this,ActionsDriver::EventType::RELEASE, release);
}

void Superkey::key_is_pressed()
{
    ActionsDriver::event_handler(this,ActionsDriver::EventType::HOLD, hold);
}

/************SUPERKEY ACTIONS*************/

uint8_t Superkey::tap()
{
    return superKeyState.tap_count++;
}

uint8_t Superkey::hold()
{
    superKeyState.holded = true;
    return true;
}

uint8_t Superkey::release()
{
    superKeyState.holded = false;
    return superKeyState.tap_count++;
}

void Superkey::timeout()
{
    superKeyState.triggered = true;
}

void Superkey::interrupt()
{
    if (superKeyState.is_interruptable){
        superKeyState.interrupt = true;
    }
}

/***************SUPERKEY TYPE****************/
void Superkey::tap_action()
{
    superKeyState.type = SuperType::Tap_Once;
}
void Superkey::hold_action()
{
    superKeyState.type = SuperType::Hold_Once;
}
void Superkey::tap_and_hold_action()
{
    superKeyState.type = SuperType::Tap_Hold;
}
void Superkey::double_tap_action()
{
    superKeyState.type = SuperType::Tap_Twice;
}
void Superkey::double_tap_hold_action()
{
    superKeyState.type = SuperType::Tap_Twice_Hold;
}
