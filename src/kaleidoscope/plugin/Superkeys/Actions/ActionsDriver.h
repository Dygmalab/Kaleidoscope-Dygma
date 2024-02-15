#ifndef NRF_NEURON_ACTIONSDRIVER_H
#define NRF_NEURON_ACTIONSDRIVER_H
#include "libraries/Kaleidoscope/src/kaleidoscope/plugin/Superkeys/includes.h"
#include <cstdint>

using EventFunction = uint8_t (*)();
class Superkey;
class ActionsDriver
{
  public:
    // Variables
/*    enum class TapType
    {
        None,
        Tap_Once,
        Hold_Once,
        Tap_Hold,
        Tap_Twice,
        Tap_Twice_Hold,
        Tap_Trice,
    };*/

/*    struct Actions
    {
        Key tap;
        Key hold;
        Key tap_hold;
        Key double_tap;
        Key double_tap_hold;
    };*/
    // Functions
    static Key return_type(uint8_t tap_count, const Key *actions);
    static bool action_handler(uint8_t tap_count,const Key *actions,const Key &key,const KeyAddr &keyAddr);
};


#endif // NRF_NEURON_ACTIONSDRIVER_H
