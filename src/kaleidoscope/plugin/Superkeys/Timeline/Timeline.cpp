/* Timeline - Timeline support for Kaleidoscope.
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
#include "Timeline.h"
#include "Superkeys/Superkey/Superkey.h"

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

                // interrupt_result=true means the superkey was interrupted and marked as released.
                // However, we should NOT remove it from the timeline yet because it hasn't finalized.
                // The superkey will be removed when disable() is called after timeout().
                // So we just mark that an interruption occurred but don't remove the entry.
                if (interrupt_result)
                {
                    interruptionOccurred = true;
                    // Don't remove or restart - let the superkey finalize in its run() cycle
                }
            }
        }
    }

    return interruptionOccurred;
}

void Timeline::process()
{

}

void Timeline::process_superkeys_in_order()
{
    // Process entries in timeline order (oldest to newest)
    // CRITICAL: Only allow ONE entry to finalize per cycle to preserve order
    // Process both superkeys and normal keys to maintain chronological order
    
    bool one_processed = false;  // Track if we've already processed one entry this cycle
    
    // Process from oldest to newest (forward iteration)
    uint8_t i = 0;
    while (i < count)
    {
        if (entries[i].type == Utils::KeyType::SUPERKEY)
        {
            Superkey* sk = static_cast<Superkey*>(entries[i].context);
            if (sk != nullptr && sk->is_enable())
            {
                uint8_t count_before = count;
                sk->run();
                
                // If the superkey called disable() and removed itself from timeline,
                // count decreased and all entries shifted left.
                if (count < count_before)
                {
                    
                    one_processed = true;
                    
                    // CRITICAL: Stop processing after first finalization
                    // This ensures entries finalize in separate cycles, preserving order
                    break;
                }
            }
        }
        else if (entries[i].type == Utils::KeyType::NORMAL)
        {
            // Normal key: check if there are any superkeys before it
            bool has_prev_sk = false;
            for (int j = i - 1; j >= 0; --j)
            {
                if (entries[j].type == Utils::KeyType::SUPERKEY)
                {
                    has_prev_sk = true;
                    break;
                }
            }
            
            if (!has_prev_sk)
            {
                // No superkeys before this normal key, send it now
                
                // Inject the key press
                handleKeyswitchEvent(entries[i].key, entries[i].addr, IS_PRESSED | INJECTED);
                
                // Remove from timeline
                remove(entries[i].addr);
                
                one_processed = true;
                
                break;
            }
        }
        i++;
    }
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
        return false;
    }

    
    // Scan backwards to see if there is any earlier SUPERKEY in the timeline
    // If a superkey is in the timeline, it's still pending regardless of enabled state
    // because it will be removed from timeline when fully processed
    for (int j = idx - 1; j >= 0; --j)
    {
        if (entries[j].type == Utils::KeyType::SUPERKEY)
        {
            Superkey* sk = static_cast<Superkey*>(entries[j].context);
            if (sk != nullptr)
            {
                return true;
            }
        }
    }
    return false;
}

Timeline timeline; // Global instance of Timeline
