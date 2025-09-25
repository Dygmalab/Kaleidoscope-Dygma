#ifndef KEY_ROLE_MANAGER_H
#define KEY_ROLE_MANAGER_H

#include "EEPROM-Settings.h"
#include "Kaleidoscope-FocusSerial.h"
#include "Kaleidoscope-Ranges.h"
#include "kaleidoscope/plugin/Superkeys/includes.h"
#include <Kaleidoscope.h>

namespace kaleidoscope
{
namespace plugin
{

class KeyRoleManager : public kaleidoscope::Plugin
{
  public:
    KeyRoleManager();

    /**
     * Handle a key switch event and perform KeyRoleManager processing.
     *
     * This method is responsible for handling key switch events and performing KeyRoleManager processing when applicable.
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
     * Handle focus events and configuration commands for KeyRoleManager.
     *
     * This method processes focus events and configuration commands related to KeyRoleManager.
     * It allows you to manage and modify various settings for KeyRoleManager, such as key mappings, wait times, timeouts, hold durations, and overlap thresholds.
     *
     * @param command A character array representing the command for configuration.
     * @return An EventHandlerResult indicating the result of the event handling.
     */
    EventHandlerResult onFocusEvent(const char *command);

    /**
     * Prepare KeyRoleManager state before reporting key events.
     *
     * This method is responsible for managing the state of KeyRoleManager just before reporting key events
     *
     * @return An EventHandlerResult indicating the result of the operation.
     */
    EventHandlerResult beforeReportingState();

    EventHandlerResult onSetup();

    /**
     * Initialize KeyRoleManager with storage settings.
     *
     * This method sets up the KeyRoleManager by configuring storage parameters.
     * It allocates a storage slice in EEPROM to store the KeyRoleManager settings, including size and offset.
     * After setting up the storage, it updates the KeyRoleManager cache to ensure consistency with the stored values.
     */
    static void setup_superkeys(uint8_t _max_layers);

    Key search_and_replace(Key key);

  private:
    static constexpr uint8_t KEYS_IN_SUPERKEY = 6;
    static constexpr uint8_t IDLE_KEY = 1;

    const uint32_t ModKeys[6] = {
        0,     //"None",
        49425, //"Dual Shift",
        49169, //"Dual Control"
        49937, // "Dual OS"
        49681, //"Dual Alt"
        50705  //"Dual Alt Gr"
    };

    const uint32_t LayerKeys[9] = {
        0,     //"None"
        51218, // "Dual Layer 1"
        51474, // "Dual Layer 2"
        51730, // "Dual Layer 3"
        51986, // "Dual Layer 4"
        52242, // "Dual Layer 5",
        52498, //"Dual Layer 6"
        52754, //"Dual Layer 7"
        53010  // "Dual Layer 8"
    };

    struct key_storage_t
    {
        // Keys configured in every superkey action.4
        Key keys[Utils::SUPER_KEY_COUNT][KEYS_IN_SUPERKEY];

        void reset()
        {
            static Key idle_key;
            idle_key.setRaw(0xFFFF);
            for (uint8_t i = 0; i < Utils::SUPER_KEY_COUNT; ++i)
            {
                for (uint8_t j = 0; j < KEYS_IN_SUPERKEY; ++j)
                {
                    keys[i][j] = idle_key;
                }
            }
        }
    };
    key_storage_t key_storage;
    key_storage_t sk_storage;
    uint8_t sk_index;

    enum class key_roles_t
    {
        NONE,
        QUKEY,
        SUPERKEY
    };

    void determine_key_role();

    void config();

    void send_sk_map();

    bool is_only_modifier(Key key);

    void dumpKeymap();

    uint16_t replace_superkey_with_qukey(const Key *action_0, const Key *action_1);

    uint16_t calculate_qukey_code(uint32_t base_raw, uint16_t selected_keycode);

    void save_sk(const Key *action_0, const Key *action_1, const Key *action_2, const Key *action_3, const Key *action_4);
};

} // namespace plugin
} // namespace kaleidoscope

extern kaleidoscope::plugin::KeyRoleManager keyRoleManager;

#endif // KEY_ROLE_MANAGER_H