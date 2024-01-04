/* Super-keys Handler - Super-keys Handler support for Kaleidoscope.*/

#ifndef NRF_NEURON_SUPERKEYSHANDLER_H
#define NRF_NEURON_SUPERKEYSHANDLER_H

#include <Kaleidoscope.h>
#include "Kaleidoscope-Ranges.h"
#include "EEPROM-Settings.h"
#include "Kaleidoscope-FocusSerial.h"

#include "libraries/Kaleidoscope/src/kaleidoscope/plugin/Superkeys/includes.h"


using KeyID = uint16_t;

namespace kaleidoscope
{
namespace plugin
{
class SuperkeysHandler : public kaleidoscope::Plugin
{
  static constexpr uint8_t SUPER_KEY_COUNT = kaleidoscope::ranges::DYNAMIC_SUPER_LAST - kaleidoscope::ranges::DYNAMIC_SUPER_FIRST + 2;
  static constexpr uint8_t MAX_SUPER_KEYS_ACTIVE = 50;
  static constexpr uint8_t KEY_PER_ACTION = 6;
  static constexpr uint8_t offset = 8;


  public:
    // Kaleidoscope plugin methods

    /**
     * Handle a key switch event and perform SuperKeys processing.
     *
     * This method is responsible for handling key switch events and performing SuperKeys processing when applicable.
     * It checks whether a key switch event corresponds to a physical key, ignores injected keys from other plugins,
     * and handles normal key events. It also manages the release of configured tap or held keys when modifiers are pressed or released.
     *
     * @param mapped_key The mapped key associated with the key switch event.
     * @param key_addr The address of the key for the key switch event.
     * @param keyState The state of the key switch event.
     * @return An EventHandlerResult indicating the result of the event handling.
     */
    EventHandlerResult onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t keyState);
    /**
     * Handle focus events and configuration commands for DynamicSuperKeys.
     *
     * This method processes focus events and configuration commands related to DynamicSuperKeys.
     * It allows you to manage and modify various settings for SuperKeys, such as key mappings, wait times, timeouts, hold durations, and overlap thresholds.
     *
     * @param command A character array representing the command for configuration.
     * @return An EventHandlerResult indicating the result of the event handling.
     */
    EventHandlerResult onFocusEvent(const char *command);
    /**
     * Prepare DynamicSuperKeys state before reporting key events.
     *
     * This method is responsible for managing the state of DynamicSuperKeys just before reporting key events.
     * It handles releasing SuperKeys and applying time-outs if necessary.
     * Additionally, it ensures that fast key releases and the state of the keys are appropriately managed.
     *
     * @return An EventHandlerResult indicating the result of the operation.
     */
    EventHandlerResult beforeReportingState();
    /**
     * Initialize DynamicSuperKeys with storage settings.
     *
     * This method sets up the DynamicSuperKeys by configuring storage parameters.
     * It allocates a storage slice in EEPROM to store the DynamicSuperKeys settings, including size and offset.
     * After setting up the storage, it updates the DynamicSuperKeys cache to ensure consistency with the stored values.
     *
     * @param dynamic_offset The offset for DynamicSuperKeys in the EEPROM storage.
     * @param size The size of the storage slice required for DynamicSuperKeys settings.
     */
    static void setup();
    static uint8_t get_active_sk();

    struct Configurations{
        //Memory space
        uint16_t storage_base_;
        uint16_t storage_size_;

        //Superkey configurations
        uint16_t delayed_time_;
        uint16_t wait_for_ ;
        uint16_t hold_start_;
        uint8_t repeat_interval_;
        uint8_t overlap_threshold_;
        uint16_t time_out_;

        //Keys
        Key keys[SUPER_KEY_COUNT][KEY_PER_ACTION];

        void reset(){
            delayed_time_ = 0;
            wait_for_ = 500;
            hold_start_ = 236;
            repeat_interval_ = 20;
            overlap_threshold_ = 80;
            time_out_ = 144;
            static Key IDLE_KEY;
            IDLE_KEY.setRaw(0xFFFF);
            for (uint16_t i = 0; i < SUPER_KEY_COUNT; ++i)
            {
                for (int j = 0; j < KEY_PER_ACTION; ++j)
                {
                    keys[i][j] = IDLE_KEY;
                }
            }
        }
    };

  private:

    static Superkey state_[SUPER_KEY_COUNT];
    static uint16_t settings_base_;
    static Superkey Sk_queue[MAX_SUPER_KEYS_ACTIVE];
    static uint8_t active_superkeys;

    static void init();
    static void config();
    static void enable();
    static void disable();
    static void sk_enabled(KeyID);
    static void save_configurations();
    static void set_active_sk();
};
} // namespace plugin
} // namespace kaleidoscope
extern kaleidoscope::plugin::SuperkeysHandler SuperkeysHandler;

#endif // NRF_NEURON_SUPERKEYSHANDLER_H
