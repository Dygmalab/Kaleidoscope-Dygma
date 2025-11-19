/* Key_List - Key_List support for Kaleidoscope.
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

#ifndef NRF_NEURON_KEY_LIST_H
#define NRF_NEURON_KEY_LIST_H

namespace KeyList
{

    struct Range
    {
        int start;
        int end;
    };

    // Utils
    uint8_t binarySearch(Range ranges[], uint8_t n, uint8_t value)
    {
        int start = 0;
        int end = n - 1;

        while (start <= end)
        {
            int mid = start + (end - start) / 2;

            if (value >= ranges[mid].start && value <= ranges[mid].end)
            {
                return mid; // Found the range
            }
            else if (value < ranges[mid].start)
            {
                end = mid - 1; // Search in the left half
            }
            else
            {
                start = mid + 1; // Search in the right half
            }
        }

        return -1; // The value is not in any range
    }
}

#endif // NRF_NEURON_KEY_LIST_H
