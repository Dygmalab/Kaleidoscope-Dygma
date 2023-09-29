/* -*- mode: c++ -*-
 * kaleidoscope::driver::storage::Flash -- Storage driver with Flash backend
 * Copyright (C) 2019  Keyboard.io, Inc
 * Copyright (C) 2019  Dygma Lab S.L.
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

/*
 * TODO(algernon): This currently uses <FlashAsEEPROM.h>, making the Props
 * struct fairly useless. At some point, we need to figure out a way to do this
 * without EEPROM API emulation, and by having the flash data variable somewhere
 * within the `storage::Flash` class.
 */

#pragma once

#include "EEPROM.h"
#include "kaleidoscope/driver/storage/Base.h"

// #include "Flash_storage.h"

namespace kaleidoscope {
namespace driver {
namespace storage {

struct FlashProps : kaleidoscope::driver::storage::BaseProps {
    static constexpr uint16_t length = 8192;
};

template <typename _StorageProps>
class Flash: public kaleidoscope::driver::storage::Base<_StorageProps> {
  public:
    template<typename T>
    T& get(uint16_t offset, T& t) {
        return EEPROM.get(offset, t);
    }

    template<typename T>
    const T& put(uint16_t offset, T& t) {
        EEPROM.put(offset, t);

        return t;
    }

    uint8_t read(int idx) {
        return EEPROM.read(idx);
    }

    void write(int idx, uint8_t val) {
        EEPROM.write(idx, val);
    }

    void update(int idx, uint8_t val) {
        EEPROM.update(idx, val);
    }

    void commit() {
        EEPROM.commit();
    }

    void begin(size_t size) {
        EEPROM.begin(size);
    }
};

}
}
}