#ifndef KEYBOARDMANAGER_H
#define KEYBOARDMANAGER_H
/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::raise2 -- Kaleidoscope device plugin for Dygma raise2
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

#ifndef __RAISE_2_H__
#define __RAISE_2_H__

#include "keyboard_config.h"

#include "kaleidoscope/device/dygma/KeyboardManager/universalModules/Hand.h"
#include "kaleidoscope/driver/bootloader/nrf/NRF.h"
#include "Ble_composite_dev.h"

#define CRGB(r, g, b) \
    (cRGB) {          \
        b, g, r, 0    \
    }

#include "Arduino.h"
#include "kaleidoscope/device/Base.h"
#include "kaleidoscope/driver/keyscanner/Base.h"
#include "kaleidoscope/driver/led/Base.h"
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

// LHK = Left Hand Keys
#define LHK 33

using kaleidoscope::driver::led::no_led;

struct KeyboardLEDDriverProps : public kaleidoscope::driver::led::BaseProps {
    static constexpr uint8_t key_matrix_leds = KEY_MATRIX_LEDS;  // Per keyboard side. ANSI only.

    static constexpr uint8_t underglow_leds_leftSide  = UNDERGLOW_LEDS_LEFT_SIDE;  //UG Left side.
    static constexpr uint8_t leds_hand_left  = LEDS_HAND_LEFT;  // BL Left side

    static constexpr uint8_t underglow_leds_rightSide  = UNDERGLOW_LEDS_RIGHT_SIDE;  // UG Right side.
    static constexpr uint8_t leds_hand_right  = LEDS_HAND_RIGHT;  // BL Right side.

    //static constexpr uint8_t leds_hand = underglow_leds + key_matrix_leds;
    static constexpr uint8_t neuron_led = NEURON_LED;

    static constexpr uint8_t leds_hand       = underglow_leds_rightSide + underglow_leds_leftSide + leds_hand_right + leds_hand_left;
    //static constexpr uint16_t led_count = leds_hand + neuron_led; //149 This has to be par number that's why the neuron takes 2
    static constexpr uint8_t led_count = leds_hand + neuron_led; //This number needs to be par so Neuron takes two LEDs
    // clang-format off
// clang-format off
 static constexpr uint8_t key_led_map[] = KEY_LED_MAP;
   // clang-format on
};

#undef LHK

class KeyboardLEDDriver : public kaleidoscope::driver::led::Base<KeyboardLEDDriverProps> {
   public:
    static void setup();

    static void syncLeds();
    static void setCrgbAt(uint8_t i, cRGB crgb);
    static void setCrgbNeuron(cRGB crgb);
    static cRGB getCrgbAt(uint8_t i);
    //Wired parameters
    static void setBrightness(uint8_t brightness);
    static uint8_t getBrightness();
    static void setBrightnessUG(uint8_t brightnessUG);
    static uint8_t getBrightnessUG();
    //Wireless parameters
    static void setBrightnessWireless(uint8_t brightness);
    static uint8_t getBrightnessWireless();
    static void setBrightnessUGWireless(uint8_t brightnessUG);
    static uint8_t getBrightnessUGWireless();
    static void updateNeuronLED();
    static constexpr uint8_t underglow_leds  = KeyboardLEDDriverProps::underglow_leds_leftSide;
    static constexpr uint8_t key_matrix_left = KeyboardLEDDriverProps::leds_hand_left;
    static constexpr uint8_t key_matrix_right = KeyboardLEDDriverProps::leds_hand_right;
    static constexpr uint8_t underglow_leds_right = KeyboardLEDDriverProps::underglow_leds_rightSide;
   private:
    static bool isLEDChangedNeuron;
    static bool leds_enabled_;
    static uint8_t isLEDChangedLeft[LED_BANKS];
    static uint8_t isLEDChangedRight[LED_BANKS];
    static cRGB neuronLED;

    static constexpr uint8_t lph = LEDS_PER_HAND;
    // clang-format off
    // led_count + 1, to account for the Neuron's LED. The last one is the
    // Neuron's LED, never send that to SLED.
    //static constexpr uint8_t led_map[][Raise2LEDDriverProps::led_count + 1] = {
//constexpr static uint8_t led_mapping_left[]={0,1,2,3,4,5,6,0xff,7,8,9,10,11,12,13,100,14,15,16,17,18,19,100,21,21,22,23,24,25,26,20,28,27,28,29,30,33,31,32,35};
    static constexpr uint8_t led_map[KeyboardLEDDriverProps::led_count] = LED_MAP;
    // clang-format on
};

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

//struct KeyboardSideFlasherProps : public kaleidoscope::util::flasher::BaseProps {};

struct KeyboardProps : kaleidoscope::device::BaseProps {
    typedef KeyboardLEDDriverProps LEDDriverProps;
    typedef KeyboardLEDDriver LEDDriver;
    typedef KeyboardKeyScannerProps KeyScannerProps;
    typedef KeyboardKeyScanner KeyScanner;
    //typedef KeyboardStorageProps StorageProps;
    typedef kaleidoscope::driver::bootloader::nrf::nrfBoot Bootloader;
    typedef kaleidoscope::driver::storage::Flash<StorageProps> Storage;

    //typedef KeyboardSideFlasherProps SideFlasherProps;
    //typedef kaleidoscope::util::flasher::KeyboardioI2CBootloader<SideFlasherProps> SideFlasher;
    static constexpr const char *short_name = SHORT_NAME;
};

class KeyboardNrf : public kaleidoscope::device::Base<KeyboardProps> {
    /*private:
  static KeyboardProps::SideFlasher SideFlasher;*/

   public:
    static void setup();

    auto serialPort() -> Stream & {
        if(ble_innited()){
            return RawHID;
        }
        return Serial;
    }

    /*auto sideFlasher() -> decltype(SideFlasher) & {
    return SideFlasher;
  }*/
    struct side {
        uint8_t getPower();
        void setPower(uint8_t power);

        uint8_t leftVersion();
        uint8_t rightVersion();

        static void reset_sides();

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


#endif // __RAISE_2_H__

#endif // ARDUINO_ARCH_NRF52
#endif //KEYBOARDMANAGER_H