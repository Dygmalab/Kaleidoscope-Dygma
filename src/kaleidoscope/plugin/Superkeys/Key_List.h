//
// Created by Urano on 9/2/2024.
//

#ifndef NRF_NEURON_KEY_LIST_H
#define NRF_NEURON_KEY_LIST_H

namespace KeyList{

struct Range {
    int start;
    int end;
};

//Utils
uint8_t binarySearch(Range ranges[], uint8_t n, uint8_t value){
    int start = 0;
    int end = n - 1;

    while (start <= end) {
        int mid = start + (end - start) / 2;

        if (value >= ranges[mid].start && value <= ranges[mid].end) {
            return mid; // Found the range
        } else if (value < ranges[mid].start) {
            end = mid - 1; // Search in the left half
        } else {
            start = mid + 1; // Search in the right half
        }
    }

    return -1; // The value is not in any range
}
}

#endif // NRF_NEURON_KEY_LIST_H
