#ifndef NRF_NEURON_TIMELINE_H
#define NRF_NEURON_TIMELINE_H
#include "kaleidoscope/plugin/Superkeys/includes.h"

class Timeline
{
public:
    Timeline();
    static constexpr uint8_t MAX_ENTRIES = 10;

    bool add(const  Utils::TimelineEntry& entry);
    void remove(const KeyAddr& addr);
    void process();
    
    // Process all superkeys in timeline order (from oldest to newest)
    void process_superkeys_in_order();
    
    // Returns true if there is any SUPERKEY entry in the timeline that
    // appears before the entry at the provided address, and is still active.
    bool has_previous_superkey_pending(const KeyAddr& addr) const;

private:

    Utils::TimelineEntry entries[MAX_ENTRIES];
    uint8_t count = 0;

    bool check_interruptions();
    bool key_is_present( Utils::TimelineEntry key );
};
extern Timeline timeline;

#endif //NRF_NEURON_TIMELINE_H
