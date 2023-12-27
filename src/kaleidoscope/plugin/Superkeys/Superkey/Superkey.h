#ifndef NRF_NEURON_SUPERKEY_H
#define NRF_NEURON_SUPERKEY_H

#include "kaleidoscope/src/kaleidoscope/plugin/Superkeys/Actions/ActionsDriver.h"
#include <Kaleidoscope.h>

class Superkey
{
  public:

    //Superkeys states
    void init();
    void config();
    void enable();
    void disable();
    void run();

    //Key actions
    void key_pressed();
    void key_released();
    void key_is_pressed();

    // Superkey Actions
    void tap_action();
    void hold_action();
    void tap_and_hold_action();
    void double_tap_action();
    void double_tap_hold_action();

  private:
    struct ActionsConfiguration
    {
        bool tap_set;
        bool hold_set;
        bool tapHold_set;
        bool doubleTap_set;
        bool doubleTapHold_set;
    };

    enum class SuperType
    {
        None,
        Tap_Once,
        Hold_Once,
        Tap_Hold,
        Tap_Twice,
        Tap_Twice_Hold,
        Tap_Trice,
    };

    struct SuperKeyState
    {
        //Sk states
        bool pressed : 1;
        bool triggered : 1;
        bool holded : 1;
        bool released : 1;
        bool interrupt : 1;
        bool remove_from_queue : 1;

        //Sk tap count
        uint8_t tap_count;
        SuperType type;
        ActionsConfiguration actionsConfiguration;

        //Sk type
        bool is_qukey;
        bool is_interruptable;

        //Timers
        uint32_t start_time;
        uint32_t hold_start;
        uint16_t delayed_time;
        uint32_t timeStamp;
    };

    bool enabled;
    uint8_t index_;
    uint8_t action_cnt;
    static Superkey::SuperKeyState superKeyState;
    static Key key_;
    static KeyAddr keyAddr_;

    //Superkey States
    static uint8_t tap();
    static uint8_t release();
    static uint8_t hold();
    static void timeout();
    static void interrupt();

};
#endif // NRF_NEURON_SUPERKEY_H
