/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::Raise -- Kaleidoscope device plugin for Dygma Raise
 * Copyright (C) 2017-2019  Keyboard.io, Inc
 * Copyright (C) 2017-2020  Dygma Lab S.L.
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
#include "Hand.h"

#include "KeyboardManager.h"
#include <cstring>

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

namespace kaleidoscope
{
namespace device
{
namespace dygma
{
namespace dygma_keyboards
{

Hand::Hand(HandSide side) : this_device_(side)
{
}

void Hand::init()
{
    /* Initialize the key data */
    keyDataReleaseAll( &key_data_ );
}

void Hand::releaseAllKeys()
{
    if( keyDataAllReleased(&key_data_) == true )
    {
        /* The keys are released already */
        return;
    }

    /* Release all keys */
    keyDataReleaseAll( &key_data_ );
    new_key_ = true;
}

void Hand::keyDataReleaseAll( key_data * p_key_data )
{
    memset( p_key_data->rows, 0x00, sizeof(p_key_data->rows) );
}

bool Hand::keyDataAllReleased( key_data * p_key_data )
{
    uint8_t i;
    for( i = 0; i < MATRIX_ROWS; i++ )
    {
        if( p_key_data->rows[i] != 0 )
        {
            return false;
        }
    }

    return true;
}

void Hand::keyDataAdd( const uint8_t * p_data, uint32_t data_len )
{
    if (memcmp(key_data_.rows, p_data, sizeof(key_data_.rows)) == 0) return;
    new_key_ = true;
    memcpy(key_data_.rows, p_data, sizeof(key_data_.rows));
}

} // namespace dygma_keyboards
} // namespace dygma
} // namespace device
} // namespace kaleidoscope
#endif
