#ifndef NRF_NEURON_TIMELINE_H
#define NRF_NEURON_TIMELINE_H
#include "libraries/Kaleidoscope/src/kaleidoscope/plugin/Superkeys/includes.h"

class Timeline
{
public:
    Timeline();
    static constexpr uint8_t MAX_ENTRIES = 10;

    bool add(const  Utils::TimelineEntry& entry);
    void remove(const KeyAddr& addr);
    void process();

private:

    Utils::TimelineEntry entries[MAX_ENTRIES];
    uint8_t count = 0;

    bool check_interruptions();
    bool key_is_present( Utils::TimelineEntry key );
};
extern Timeline timeline;

#endif //NRF_NEURON_TIMELINE_H
