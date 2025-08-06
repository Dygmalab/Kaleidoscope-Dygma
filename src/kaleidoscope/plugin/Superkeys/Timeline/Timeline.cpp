#include "Timeline.h"

Timeline::Timeline() : count(0)
{
    Utils::TimelineEntry emptyEntry =
    {
            Key(0xFFFF), // Invalid key
            KeyAddr(0xff, 0xff), // Invalid address
            0, // Timestamp
            Utils::KeyType::NONE, // Type
            false, // is_interruptible
            nullptr // context
    };

    for (uint8_t i = 0; i < MAX_ENTRIES; i++)
    {
        entries[i] =emptyEntry;
    }
}

bool Timeline::add(const Utils::TimelineEntry& entry)
{
    if (count < MAX_ENTRIES && !key_is_present(entry) )
    {
       // NRF_LOG_DEBUG("*************Adding Key to timeline: *************");
        entries[count++] = entry;

/*        for (uint8_t i = 0; i < count; i++)
        {
            NRF_LOG_DEBUG("Entry %d: Key: %d, Type: %d", i, entries[i].key.getRaw(), static_cast<int>(entries[i].type));
        }*/
        return check_interruptions();
    }
    return false;
}

bool Timeline::key_is_present( Utils::TimelineEntry entry_key )
{
    for (uint8_t i = 0; i < count; i++)
    {
        if (entries[i].key == entry_key.key && entries[i].addr == entry_key.addr)
        {
            return true;
        }
    }
    return false;
}

void Timeline::remove(const KeyAddr& addr) // We use the address because superkeys can change its key in runtime.
{
/*    NRF_LOG_DEBUG("*************Removing Key from timeline: *************:");*/
    for (uint8_t i = 0; i < count; i++)
    {
        if (entries[i].addr == addr)
        {
            for (uint8_t j = i; j < count - 1; j++)
            {
                entries[j] = entries[j + 1];
            }
            count--;
            return;
        }
    }
/*    for (uint8_t i = 0; i < count; i++)
    {
        NRF_LOG_DEBUG("Entry %d: Key: %d, Type: %d", i, entries[i].key.getRaw(), static_cast<int>(entries[i].type));
    }*/
}
// Count 3
//   q      w    e
// [ 0 ] [ 1 ] [ 2 ]
//  prev current next
// qwe
bool Timeline::check_interruptions()
{
    if (count < 2) return false;

    bool interruptionOccurred = false;
    constexpr uint8_t previous_key_index = 0;
    constexpr uint8_t current_key_index = 1;

    while (true)
    {
        if (count < 2) break;  // No hay suficiente para comparar

        Utils::TimelineEntry& curr = entries[current_key_index];
        bool foundInterruption = false;

        // Buscar una SK previa interrumpible
        Utils::TimelineEntry& prev = entries[previous_key_index];

        if (prev.type == Utils::KeyType::SUPERKEY)
        {
            Superkey* sk_prev = static_cast<Superkey*>(prev.context);

            if ((curr.type == Utils::KeyType::NORMAL || curr.type == Utils::KeyType::SUPERKEY))
            {
/*                NRF_LOG_DEBUG("Superkey %d is being interrupted by key %d",
                              sk_prev->get_index(), curr.key.getRaw());*/
                bool interrupt_result = sk_prev->interrupt(curr.key, curr.addr);
                if( !interrupt_result ) // If the interrupt is true the superkey will delete itself from the timeline. Otherwise we will need to remove it.
                {
                    remove(prev.addr);
                }
                interruptionOccurred = true;
                foundInterruption = true;
            }
        }

        if (!foundInterruption) break;  // No hay más interrupciones posibles
    }

    return interruptionOccurred;
}


void Timeline::process()
{

}

Timeline timeline; // Instancia global de Timeline
