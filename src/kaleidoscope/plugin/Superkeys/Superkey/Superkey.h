#ifndef NRF_NEURON_SUPERKEY_H
#define NRF_NEURON_SUPERKEY_H
#include "kaleidoscope/plugin/Superkeys/includes.h"

constexpr uint8_t KEYS_IN_SUPERKEY = 6;
constexpr uint8_t QUKEY_MIN_IDLE_ACTIONS = 3;
constexpr uint8_t IDLE_KEY = 1;

// Forward declarations.
class ActionsDriver;

class Superkey
{
public:
  // Superkeys states
  void init(const Key *act);
  void enable( uint8_t modifiers_pressed);
  void disable();
  void run();
  bool interrupt(Key &key, const KeyAddr &keyaddr);
  bool is_enable() const;
  void init_timer();
  uint16_t get_index() const;
  bool is_qukey() const;

  // Key actions
  void key_pressed();
  void key_released();
  void key_is_pressed();
  bool is_interruptible();
  Key get_phisical_key() const;
  KeyAddr get_keyAddr() const;
  void set_key_and_keyAddr(Key key, KeyAddr keyAddr);

  // Constructor
  explicit Superkey(uint16_t index, uint16_t hold_start, uint16_t time_out, uint16_t minimum_hold_start) : index_(index), time_out_(time_out), hold_start_(hold_start), minimum_hold_start_(minimum_hold_start)
  {
  }

private:
  Key phisical_key_;
  KeyAddr keyaddr_;

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
    Utils::TapType type{Utils::TapType::None};

    // Sk type
    bool is_qukey{false};
    bool is_interruptable{false};
    bool is_repeateable{false};

    // Timers
    uint32_t start_time{0};
    uint32_t hold_start{0};
    uint32_t timeStamp{0};
    uint32_t minimum_hold{0};

    // keys in Actions
    Utils::Actions action{};

    // Active external modifiers
    uint8_t cache_modifiers{0}; // This is used to cache the modifiers that are active when the superkey is pressed.
  };
  SuperKeyState superKeyState{};

  uint8_t index_{};
  uint16_t time_out_{255};
  uint16_t hold_start_{255};
  uint16_t minimum_hold_start_{100};

  // Superkey States
  void tap();
  void release();
  void hold();
  void timeout();

  /**
   * @brief Send the key to the OS.
   *
   */
  void send_key() const;

  // Sk Configurations
  void set_up_actions(const Key *act);

  /**
   * @brief Check if the superkey is a Qukey.
   *
   * Qukey superkeys are superkeys that have only two action configured, so they
   * are fast to trigger and release.
   * This method checks if the superkey has only two actions configured, and if so, sets the `is_qukey` state to true.
   */
  void check_if_sk_qukey();

  void check_if_sk_interruptable(const Key &Action);

  void set_repeated_actions(const Key &Action, bool holded)
  {
    superKeyState.action.tap = Action;
    superKeyState.is_repeateable = holded;
  }

  void keep_sending_hold_key(bool holded);

  // Utils
  void update_timestamp();

public:
  Key Actions[6] = {
      superKeyState.action.tap,
      superKeyState.action.hold,
      superKeyState.action.tap_hold,
      superKeyState.action.double_tap,
      superKeyState.action.double_tap_hold};
};

#endif // NRF_NEURON_SUPERKEY_H
