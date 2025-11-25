#ifndef KEYBOARDMANAGER_H
#define KEYBOARDMANAGER_H
/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::keyboard -- Kaleidoscope device plugin for Dygma keyboards
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

#pragma once

#include "keyboard_config.h"

#include "kaleidoscope/device/dygma/keyboardManager/universalModules/Hand.h"
#include "kaleidoscope/driver/bootloader/nrf/NRF.h"
#include "Ble_composite_dev.h"

#include "Arduino.h"
#include "kaleidoscope/device/Base.h"
#include "kaleidoscope/driver/keyscanner/Base.h"
#include "kaleidoscope/driver/storage/Flash.h"
#include "libraries/KeyboardioHID/src/MultiReport/RawHID.h"


#define KEYBOARD_HANDS_DEBUG                    1
#define PRINT_KEYSWITCH_EVENT_PARAMETERS    0

#ifndef KEYBOARD_NEURON_FW_VERSION
#error "Firmware version is not specified."
    #define KEYBOARD_NEURON_FW_VERSION "N/A"
#endif

namespace kaleidoscope {
namespace device {
namespace dygma {

struct KeyboardKeyScannerProps : public kaleidoscope::driver::keyscanner::BaseProps {
    static constexpr uint8_t matrix_rows    = MATRIX_ROWS;
    static constexpr uint8_t matrix_columns = MATRIX_COLS;
    typedef MatrixAddr<matrix_rows, matrix_columns> KeyAddr;

    static constexpr uint8_t left_columns  = LEFT_COLUMNS;
    static constexpr uint8_t right_columns = matrix_columns - left_columns;
};

class KeyboardKeyScanner : public kaleidoscope::driver::keyscanner::Base<KeyboardKeyScannerProps> {
   private:
    typedef KeyboardKeyScanner ThisType;
    typedef KeyboardKeyScannerProps Props_;

   public:
    static void setup();
    static void scanMatrix();
    static void readMatrix();
    static void actOnMatrixScan();
    static Communications_protocol::Devices rightHandDevice(void);
    static bool rightSideWiredConnection();
    static bool leftSideWiredConnection();
    static Communications_protocol::Devices leftHandDevice(void);

    static void maskKey(KeyAddr key_addr);
    static void unMaskKey(KeyAddr key_addr);
    static bool isKeyMasked(KeyAddr key_addr);
    static void maskHeldKeys();

    static bool isKeyswitchPressed(KeyAddr key_addr);
    static uint8_t pressedKeyswitchCount();

    static bool wasKeyswitchPressed(KeyAddr key_addr);
    static uint8_t previousPressedKeyswitchCount();
    static void reset(void);

   protected:
    static dygma_keyboards::key_data leftHandState;
    static dygma_keyboards::key_data rightHandState;
    static dygma_keyboards::key_data previousLeftHandState;
    static dygma_keyboards::key_data previousRightHandState;

    static dygma_keyboards::key_data leftHandMask;
    static dygma_keyboards::key_data rightHandMask;
    static void usbConnectionsStateMachine();
};

struct KeyboardProps : kaleidoscope::device::BaseProps {
    typedef KeyboardKeyScannerProps KeyScannerProps;
    typedef KeyboardKeyScanner KeyScanner;
    typedef kaleidoscope::driver::bootloader::nrf::nrfBoot Bootloader;
    typedef kaleidoscope::driver::storage::Flash<StorageProps> Storage;

    static constexpr const char *short_name = SHORT_NAME;
};

class KeyboardNrf : public kaleidoscope::device::Base<KeyboardProps> {

   public:
    static void setup();

    auto serialPort() -> Stream & {
        if(ble_innited()){
            return RawHID;
        }
        return Serial;
    }

    struct side {
        uint8_t getPower();
        void setPower(uint8_t power);

        uint8_t leftVersion();
        uint8_t rightVersion();

        static void reset_sides();

        static void reset_right_side();

        static void reset_left_side();

        void prepareForFlash();

        // Side bootloader addresses
        static constexpr uint8_t left_boot_address  = 0x5A;
        static constexpr uint8_t right_boot_address = 0x5B;
    } side;

    struct settings {
        uint16_t keyscanInterval();
        void keyscanInterval(uint16_t interval);
        void getChipID(char *buff, uint16_t len);
        void get_chip_info(char *buff, uint16_t len);
    } settings;
};


}  // namespace dygma
}  // namespace devicee

typedef kaleidoscope::device::dygma::KeyboardNrf Device;

}  // namespace kaleidoscope

#endif // ARDUINO_ARCH_NRF52
#endif //KEYBOARDMANAGER_H
