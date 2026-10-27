/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::DefyNrf -- Kaleidoscope device plugin for Dygma DefyNrf
 * Copyright (C) 2017-2020  Keyboard.io, Inc
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

#include "kaleidoscope/Runtime.h"
#include "kaleidoscope/driver/keyscanner/Base_Impl.h"
#include "KeyboardManager.h"

namespace kaleidoscope
{
namespace device
{
namespace dygma
{

/********* KeyboardHands *********/

struct KeyboardHands
{
    static dygma_keyboards::Hand leftHand;
    static dygma_keyboards::Hand rightHand;

    static void setup();
};

dygma_keyboards::Hand KeyboardHands::leftHand(dygma_keyboards::Hand::LEFT);
dygma_keyboards::Hand KeyboardHands::rightHand(dygma_keyboards::Hand::RIGHT);

void KeyboardHands::setup()
{
    rightHand.init();
    leftHand.init();
}

/********* Key scanner *********/

dygma_keyboards::key_data KeyboardKeyScanner::leftHandState;
dygma_keyboards::key_data KeyboardKeyScanner::rightHandState;
dygma_keyboards::key_data KeyboardKeyScanner::previousLeftHandState;
dygma_keyboards::key_data KeyboardKeyScanner::previousRightHandState;
dygma_keyboards::key_data KeyboardKeyScanner::leftHandMask;
dygma_keyboards::key_data KeyboardKeyScanner::rightHandMask;

void KeyboardKeyScanner::scanMatrix()
{
    readMatrix();
    actOnMatrixScan();
}

void KeyboardKeyScanner::readMatrix()
{
    previousLeftHandState = leftHandState;
    previousRightHandState = rightHandState;

    if (KeyboardHands::leftHand.newKey())
    {
        leftHandState = KeyboardHands::leftHand.getKeyData();
    }
    if (KeyboardHands::rightHand.newKey())
    {
        rightHandState = KeyboardHands::rightHand.getKeyData();
    }
}

void KeyboardKeyScanner::actOnMatrixScan()
{
    for (uint8_t row = 0; row < Props_::matrix_rows; row++)
    {
        for (uint8_t col = 0; col < HAND_COLUMN_BITMAP_BIT_SIZE; col++)
        {
            uint32_t keynum = (row * HAND_COLUMN_BITMAP_BIT_SIZE) + col;
            uint8_t keyStatePrev;
            uint8_t keyStateNow;
            uint8_t keyState;

            // left
            keyStatePrev = array_bit_get( (uint8_t *)previousLeftHandState.rows, sizeof(previousLeftHandState.rows), keynum );
            keyStateNow = array_bit_get( (uint8_t *)leftHandState.rows, sizeof(leftHandState.rows), keynum );

            keyState = (keyStatePrev << 0) | (keyStateNow << 1);

            if (keyState)
            {
#if PRINT_KEYSWITCH_EVENT_PARAMETERS
                NRF_LOG_INFO("---------------");
                NRF_LOG_INFO("Left keystroke:");
                NRF_LOG_INFO("row: %d", row);
                NRF_LOG_INFO("col: %d", col);
                NRF_LOG_INFO("key state: %d", keyState);
                NRF_LOG_FLUSH();
#endif

                ThisType::handleKeyswitchEvent(Key_NoKey, KeyAddr(row, col), keyState);
            }

            // right
            keyStatePrev = array_bit_get( (uint8_t *)previousRightHandState.rows, sizeof(previousRightHandState.rows), keynum );
            keyStateNow = array_bit_get( (uint8_t *)rightHandState.rows, sizeof(rightHandState.rows), keynum );

            keyState = (keyStatePrev << 0) | (keyStateNow << 1);

            if (keyState)
            {
#if PRINT_KEYSWITCH_EVENT_PARAMETERS
                NRF_LOG_INFO("---------------");
                NRF_LOG_INFO("Right keystroke:");
                NRF_LOG_INFO("row: %d", row);
                NRF_LOG_INFO("col: %d", col);
                NRF_LOG_INFO("key state: %d", keyState);
                NRF_LOG_FLUSH();
                NRF_LOG_FLUSH();
#endif

                ThisType::handleKeyswitchEvent(Key_NoKey, KeyAddr(row, (Props_::matrix_columns - 1) - col), keyState);
            }
        }
    }
}

void KeyboardKeyScanner::maskKey(KeyAddr key_addr)
{
    if (!key_addr.isValid()) return;

    auto row = key_addr.row();
    auto col = key_addr.col();

    if (col >= Props_::left_columns)
    {
        rightHandMask.rows[row] |= 1 << (Props_::right_columns - (col - Props_::left_columns));
    }
    else
    {
        leftHandMask.rows[row] |= 1 << (Props_::right_columns - col);
    }
}

void KeyboardKeyScanner::unMaskKey(KeyAddr key_addr)
{
    if (!key_addr.isValid()) return;

    auto row = key_addr.row();
    auto col = key_addr.col();

    if (col >= Props_::left_columns)
    {
        rightHandMask.rows[row] &= ~(1 << (Props_::right_columns - (col - Props_::left_columns)));
    }
    else
    {
        leftHandMask.rows[row] &= ~(1 << (Props_::right_columns - col));
    }
}

bool KeyboardKeyScanner::isKeyMasked(KeyAddr key_addr)
{
    if (!key_addr.isValid()) return false;

    auto row = key_addr.row();
    auto col = key_addr.col();

    if (col >= 8)
    {
        return rightHandMask.rows[row] & (1 << (7 - (col - 8)));
    }
    else
    {
        return leftHandMask.rows[row] & (1 << (7 - col));
    }
}

void KeyboardKeyScanner::maskHeldKeys()
{
    memcpy(leftHandMask.rows, leftHandState.rows, sizeof(leftHandMask));
    memcpy(rightHandMask.rows, rightHandState.rows, sizeof(rightHandMask));
}

bool KeyboardKeyScanner::isKeyswitchPressed(KeyAddr key_addr)
{
    auto row = key_addr.row();
    auto col = key_addr.col();

    if (col >= Props_::left_columns)
    {
        return (bitRead(rightHandState.rows[row], (Props_::matrix_columns - 1) - col) != 0);
    }
    else
    {
        return (bitRead(leftHandState.rows[row], col) != 0);
    }
}

bool KeyboardKeyScanner::wasKeyswitchPressed(KeyAddr key_addr)
{
    auto row = key_addr.row();
    auto col = key_addr.col();

    if (col >= Props_::left_columns)
    {
        return (bitRead(previousRightHandState.rows[row], (Props_::matrix_columns - 1) - col) != 0);
    }
    else
    {
        return (bitRead(previousLeftHandState.rows[row], col) != 0);
    }
}

uint8_t KeyboardKeyScanner::pressedKeyswitchCount()
{
    return array_popcount_get( (uint8_t *)leftHandState.rows, sizeof(leftHandState.rows) ) +
            array_popcount_get( (uint8_t *)rightHandState.rows, sizeof(rightHandState.rows) );
}

uint8_t KeyboardKeyScanner::previousPressedKeyswitchCount()
{
    return array_popcount_get( (uint8_t *)previousLeftHandState.rows, sizeof(previousLeftHandState.rows) ) +
            array_popcount_get( (uint8_t *)previousRightHandState.rows, sizeof(previousRightHandState.rows) );
}

void KeyboardKeyScanner::setup()
{
    static constexpr uint8_t keyscanner_pins[] = {2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                                                  21, 22, 23, 24, 25, 26, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42};

    for (uint32_t i = 0; i < sizeof(keyscanner_pins); i++)
    {
        // pinMode(keyscanner_pins[i], OUTPUT);
        // digitalWrite(keyscanner_pins[i], LOW);
    }
}

void KeyboardKeyScanner::reset(void)
{
    dygma_keyboards::Hand::keyDataReleaseAll( &leftHandState );
    dygma_keyboards::Hand::keyDataReleaseAll( &rightHandState );
    Runtime.hid().keyboard().releaseAllKeys();
    Runtime.hid().keyboard().sendReport();
}

/********* KeyboardNrf class (Hardware plugin) *********/

void KeyboardNrf::setup()
{
    KeyboardHands::setup();
    KeyScanner::setup();
}

result_t KeyboardNrf::key_data_add( kbdapi_side_type_t side_type, const uint8_t * p_data, uint32_t data_len )
{
    if( side_type == KBDAPI_SIDE_TYPE_LEFT )
    {
        KeyboardHands::leftHand.keyDataAdd( p_data, data_len );

        return RESULT_OK;
    }
    else if( side_type == KBDAPI_SIDE_TYPE_RIGHT )
    {
        KeyboardHands::rightHand.keyDataAdd( p_data, data_len );

        return RESULT_OK;
    }

    return RESULT_ERR;
}

} // namespace dygma
} // namespace device
} // namespace kaleidoscope


#endif  // ARDUINO_ARCH_NRF52
