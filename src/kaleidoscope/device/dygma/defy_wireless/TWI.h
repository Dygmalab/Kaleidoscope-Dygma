/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::Defy -- Kaleidoscope device plugin for Dygma Defy
 * Copyright (C) 2017-2019  Keyboard.io, Inc
 * Copyright (C) 2017-2019  Dygma Lab S.L.
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
#ifdef ARDUINO_ARCH_NRF52

#pragma once

#include <stdlib.h>
#include <stdint.h>


#define TWI_DEBUG false

#define TWI_PRINT_RECEIVED_BUFFER_ALWAYS        false
#define TWI_PRINT_RECEIVED_BUFFER_IF_KEYSTROKE  false

#define TWI_PRINT_CRC16_ALWAYS                  false
#define TWI_PRINT_CRC16_IF_ERROR                false
#define TWI_PRINT_CRC16_IF_ERROR_AND_KEYSTROKE  false


#define TWI_SCL_PIN 5
#define TWI_SDA_PIN 4
#define TWI_TIMEOUT_TIME 10


namespace kaleidoscope {
namespace device {
namespace dygma {
namespace defy_wireless {

class TWI
{
    public:
        TWI(int8_t _slave_addr) : slave_addr(_slave_addr) {};

        void init(uint16_t clock_khz_);

        uint8_t readFrom(uint8_t *buffer, uint16_t len);
        uint8_t writeTo(uint8_t *buffer, uint16_t len);

        bool check_crc16(uint8_t *buffer, uint16_t len);
        void recovery(void);
        uint8_t crc_errors(void);

    private:
        int8_t slave_addr;
        uint8_t crc_errors_ = 0;
        uint32_t clock_khz_;
        static bool twi_already_init;
};

}
}
}
}

#endif