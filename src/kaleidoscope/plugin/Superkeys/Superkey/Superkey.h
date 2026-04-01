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

#ifndef NRF_NEURON_SUPERKEY_H
#define NRF_NEURON_SUPERKEY_H

#include "kbd_core.h"
#include "Superkeys/includes.h"

constexpr uint8_t ACTIONS_IN_SUPERKEY = 6;
constexpr uint8_t QUKEY_MIN_IDLE_ACTIONS = 3;
constexpr uint8_t IDLE_KEY = 1;

class Superkey
{
public:

    typedef struct PACK
    {
        uint8_t flags;
        uint8_t keyCode;
    } action_config_t;

    typedef struct PACK
    {
        action_config_t actions[ACTIONS_IN_SUPERKEY];
    } superkey_config_t;

public:
  // Superkeys states
  void init(const superkey_config_t *p_superkey_config);
  void enable( uint8_t modifiers_pressed);
  void disable();
  void run();
  bool interrupt(Key &key, const KeyAddr &keyaddr);
  bool is_enable() const;
  bool is_holded() const;
  bool is_triggered() const;
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

  static uint16_t getActionRaw( const action_config_t * p_action_config );
  static void setActionRaw( action_config_t * p_action_config, uint16_t raw );
  static Key getActionAsKey( const action_config_t * p_action_config );
  static Key getActionAsKey( const superkey_config_t * p_superkey, uint16_t action_id );

  Superkey();
  
  // Constructor
  explicit Superkey(uint16_t index, const Utils::SharedConfig* config, const superkey_config_t * superkey_config) :
  index_(index), shared_config_(config), superkey_config_(superkey_config)
  {
  }

private:
  Key phisical_key_;
  KeyAddr keyaddr_;

  struct SuperKeyState
  {
    // Sk states (compacted as bitfields to save memory - all fit in 1 byte)
    uint8_t pressed : 1;
    uint8_t triggered : 1;
    uint8_t holded : 1;
    uint8_t released : 1;
    uint8_t interrupt : 1;
    uint8_t enabled : 1;
    uint8_t is_qukey : 1;
    uint8_t is_interruptable : 1;

    // Sk tap count and type (tap_count uses 4 bits, type uses 4 bits)
    uint8_t tap_count : 4;  // Max 15 taps (more than enough)
    Utils::TapType type : 4; // 4 bits for enum (max 16 values)

    // Timers
    uint32_t hold_start{0};
    uint32_t timeStamp{0};

    // Active external modifiers
    uint8_t cache_modifiers{0}; // This is used to cache the modifiers that are active when the superkey is pressed.
  };
  SuperKeyState superKeyState{};

  uint8_t index_{};
  const Utils::SharedConfig* shared_config_{nullptr}; // Pointer to shared configuration (4 bytes instead of 6)
  const superkey_config_t * superkey_config_{nullptr}; // Pointer to actions array (stored separately to save memory)

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
  void set_up_superkey_config(const superkey_config_t * p_superkey_config);

  /**
   * @brief Check if the superkey is a Qukey.
   *
   * Qukey superkeys are superkeys that have only two action configured, so they
   * are fast to trigger and release.
   * This method checks if the superkey has only two actions configured, and if so, sets the `is_qukey` state to true.
   */
  void check_if_sk_qukey();

  void check_if_sk_interruptable(const Key &Action);

  // Utils
  void update_timestamp();
};

#endif // NRF_NEURON_SUPERKEY_H
