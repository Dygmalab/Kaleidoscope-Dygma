#ifndef NRF_NEURON_SUPERKEY_H
#define NRF_NEURON_SUPERKEY_H
#include "libraries/Kaleidoscope/src/kaleidoscope/plugin/Superkeys/includes.h"
#include <Kaleidoscope.h>
#include <Kaleidoscope-Ranges.h>

constexpr uint8_t KEYS_IN_SUPERKEY = 6;
constexpr uint8_t QUKEY_MIN_IDLE_ACTIONS = 3;
// Forward declarations.
class ActionsDriver;

class Superkey
{
  public:
    // Superkeys states
    void init(const Key *act);
    void config();
    void enable();
    void disable();
    void run();
    bool is_enable() const;
    void init_timer();
    uint16_t get_index() const;

    // Key actions
    void key_pressed();
    void key_released();
    void key_is_pressed();

    // Superkey Actions
    void set_tap_action();
    void set_hold_action();
    void set_tap_and_hold_action();
    void set_double_tap_action();
    void set_double_tap_hold_action();

    // Constructor
    explicit Superkey(uint16_t index, uint16_t hold_start, uint16_t time_out) : index_(index), time_out_(time_out), hold_start_(hold_start)
    {
    }

    enum : uint16_t
    {
        ALPHA_KEYS = 255,
        ALPHA_WITH_MODIFIERS_FIRST = 256,
        ALPHA_WITH_MODIFIERS_LAST = 7935,
        LED_BUTTONS_FIRST = 17152,
        PREVIOUS_LED_EFFECT,
        LED_BUTTONS_LAST,
        LAYER_SHIFT_FIRST = 17450,
        LAYER_SHIFT_LAST = 17459,
        LAYER_LOCK_FIRST = 17492,
        LAYER_LOCK_LAST = 17501
    };

  private:
    enum class KeyRanges
    {
        LAYER_LOCK,
        LED_BUTTONS,
        DYNAMIC_MACRO,
        ALPHA_WITH_MODIFIERS,
        ALPHA_KEYS,
        LAYER_SHIFT,
        UNKNOW
    };

    enum class TapType
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
        // Sk states
        bool pressed{false};
        bool triggered{false};
        bool holded{false};
        bool released{false};
        bool interrupt{false};
        bool enabled{false};

        // Sk tap count
        uint8_t tap_count{0};
        TapType type{TapType::None};

        // Sk type
        bool is_qukey{false};
        bool is_interruptable{false};

        // Timers
        uint32_t start_time{0};
        uint32_t hold_start{0};
        bool is_being_hold{false};
        uint32_t timeStamp{0};

        // keys in Actions
        struct Actions
        {
            Key tap;
            Key hold;
            Key tap_hold;
            Key double_tap;
            Key double_tap_hold;
        };
        Actions Action{};

        Key Actions[6] = {Action.tap, Action.hold, Action.tap_hold, Action.double_tap, Action.double_tap_hold};
    };
    SuperKeyState superKeyState{};

    uint8_t index_{};
    uint16_t time_out_{255};
    uint16_t hold_start_{255};
    uint8_t action_cnt{0};

    // Superkey States
    uint8_t tap();
    uint8_t release();
    uint8_t hold();
    void timeout();
    void interrupt();

    void set_up_actions(const Key *act);
    void check_if_sk_qukey();
    void check_if_sk_interruptable(const Key& Action);

    //Utils
    static uint16_t find_key_type(uint16_t value);
    void update_timestamp();


  public:
    struct Range
    {
        uint16_t start;
        uint16_t end;
    };
};

#endif // NRF_NEURON_SUPERKEY_H
