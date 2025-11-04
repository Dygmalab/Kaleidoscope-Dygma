#include "Timeline.h"

#define LOG_TIMELINE 0

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
#if LOG_TIMELINE
        for (uint8_t i = 0; i < count; i++)
        {
            NRF_LOG_DEBUG("Entry %d: Key: %d, Type: %d", i, entries[i].key.getRaw(), static_cast<int>(entries[i].type));
        }
#endif
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
#if LOG_TIMELINE
    NRF_LOG_DEBUG("*************Removing Key from timeline: *************:");
#endif
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
#if LOG_TIMELINE
    for (uint8_t i = 0; i < count; i++)
    {
        NRF_LOG_DEBUG("Entry %d: Key: %d, Type: %d", i, entries[i].key.getRaw(), static_cast<int>(entries[i].type));
    }
#endif
}

bool Timeline::check_interruptions()
{
    if (count < 2) return false;

    bool interruptionOccurred = false;

    // We start from the end towards the beginning to respect the order of key presses
    for (int i = count - 2; i >= 0; i--)
    {
        Utils::TimelineEntry& prev = entries[i];
        Utils::TimelineEntry& curr = entries[i + 1];

        if (prev.type == Utils::KeyType::SUPERKEY)
        {
            Superkey* sk_prev = static_cast<Superkey*>(prev.context);

            if (curr.type == Utils::KeyType::NORMAL || curr.type == Utils::KeyType::SUPERKEY)
            {
                bool interrupt_result = sk_prev->interrupt(curr.key, curr.addr);

                // Only when the previous superkey DID handle the interruption
                // (i.e. it finalized itself), we remove it from the timeline
                // and restart the analysis to keep references valid.
                if (interrupt_result)
                {
                    remove(prev.addr);
                    interruptionOccurred = true;
                    // Since we removed an element, we start the analysis again
                    return check_interruptions();
                }
            }
        }
    }

    return interruptionOccurred;
}

void Timeline::process()
{

}

bool Timeline::has_previous_superkey_pending(const KeyAddr& addr) const
{
    // Find the index of the entry with the given address
    int8_t idx = -1;
    for (uint8_t i = 0; i < count; ++i)
    {
        if (entries[i].addr == addr)
        {
            idx = static_cast<int8_t>(i);
            break;
        }
    }

    if (idx <= 0)
    {
        //NRF_LOG_DEBUG("has_previous_superkey_pending: idx=%d, returning false", idx);
        return false;
    }

    //NRF_LOG_DEBUG("has_previous_superkey_pending: checking %d entries before idx=%d", idx, idx);
    
    // Scan backwards to see if there is any earlier SUPERKEY still enabled
    for (int j = idx - 1; j >= 0; --j)
    {
        if (entries[j].type == Utils::KeyType::SUPERKEY)
        {
            Superkey* sk = static_cast<Superkey*>(entries[j].context);
            bool is_enabled = (sk != nullptr && sk->is_enable());
            //NRF_LOG_DEBUG("  Entry[%d]: SUPERKEY at %d:%d, enabled=%d", 
            //              j, entries[j].addr.row(), entries[j].addr.col(), is_enabled);
            if (is_enabled)
            {
                //NRF_LOG_DEBUG("has_previous_superkey_pending: found enabled superkey, returning true");
                return true;
            }
        }
        else
        {
            //NRF_LOG_DEBUG("  Entry[%d]: type=%d (not SUPERKEY)", j, static_cast<int>(entries[j].type));
        }
    }

    //NRF_LOG_DEBUG("has_previous_superkey_pending: no enabled superkeys found, returning false");
    return false;
}

Timeline timeline; // Global instance of Timeline