/* Super-keys Handler - Super-keys Handler support for Kaleidoscope.*/

#ifndef NRF_NEURON_SUPERKEYSHANDLER_H
#define NRF_NEURON_SUPERKEYSHANDLER_H

#include "libraries/Kaleidoscope/src/kaleidoscope/plugin/Superkeys/Superkey/Superkey.h"
#include <Kaleidoscope.h>

using KeyID = uint16_t;

namespace kaleidoscope
{
namespace plugin
{
class SuperkeysHandler : public kaleidoscope::Plugin
{
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
    static void setup(uint8_t dynamic_offset, uint16_t size);

  public:
    // Variable declarations
  private:
    //Superkey Sk_queue[15];

  private:
    static void init();
    static void config();
    static void enable();
    static void disable();
    static void run();
    static void sk_enabled(KeyID);
};

} // namespace plugin
} // namespace kaleidoscope
extern kaleidoscope::plugin::SuperkeysHandler SuperkeysHandler;

#endif // NRF_NEURON_SUPERKEYSHANDLER_H
