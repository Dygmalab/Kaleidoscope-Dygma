/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::Defy -- Kaleidoscope device plugin for Dygma Defy
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

#ifndef __DEFY_WIRELESS_H__
#define __DEFY_WIRELESS_H__

#include "kaleidoscope/device/dygma/keyboards/Hand.h"
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
#define LHK 35

using kaleidoscope::driver::led::no_led;

struct KeyboardLEDDriverProps : public kaleidoscope::driver::led::BaseProps {
    static constexpr uint8_t underglow_leds  = 53;
    static constexpr uint8_t key_matrix_leds = 35;

    static constexpr uint8_t underglow_leds_leftSide  = 53;  //UG Left side.
    static constexpr uint8_t leds_hand_left  = 35;  // BL Left side

    static constexpr uint8_t underglow_leds_rightSide  = 53;  // UG Right side.
    static constexpr uint8_t leds_hand_right  = 35;  // BL Right side.

    //static constexpr uint8_t leds_hand = underglow_leds + key_matrix_leds;
    static constexpr uint8_t neuron_led = 2;

    static constexpr uint8_t leds_hand       = underglow_leds_rightSide + underglow_leds_leftSide + leds_hand_right + leds_hand_left;
    //static constexpr uint16_t led_count = leds_hand + neuron_led; //178 This has to be par number that's why the neuron takes 2
    static constexpr uint8_t led_count = leds_hand + neuron_led; //This number needs to be par so Neuron takes two LEDs


    // clang-format off
  static constexpr uint8_t key_led_map[] = {
	  // ISO & ANSI (ANSI has no LED at 20, but this key can never be pressed so we can have just one map).
	  0, 1, 2, 3, 4, 5, 6, no_led, no_led, 6 + key_matrix_leds, 5 + key_matrix_leds, 4 + key_matrix_leds, 3 + key_matrix_leds, 2 + key_matrix_leds, 1 + key_matrix_leds, 0 + key_matrix_leds,
	  7, 8, 9, 10, 11, 12, 13, no_led, no_led, 13 + key_matrix_leds, 12 + key_matrix_leds, 11 + key_matrix_leds, 10 + key_matrix_leds, 9 + key_matrix_leds, 8 + key_matrix_leds, 7 + key_matrix_leds,
	  14, 15, 16, 17, 18, 19, 20, no_led, no_led, 20 + key_matrix_leds, 19 + key_matrix_leds, 18 + key_matrix_leds, 17 + key_matrix_leds, 16 + key_matrix_leds, 15 + key_matrix_leds, 14 + key_matrix_leds,
	  21, 22, 23, 24, 25, 26, no_led, no_led, no_led, no_led, 26 + key_matrix_leds, 25 + key_matrix_leds, 24 + key_matrix_leds, 23 + key_matrix_leds, 22 + key_matrix_leds,
	  21 + key_matrix_leds,
	  27, 28, 29, 30, 31, 32, 33, 34, 34 + key_matrix_leds, 33 + key_matrix_leds, 32 + key_matrix_leds, 31 + key_matrix_leds, 30 + key_matrix_leds, 29 + key_matrix_leds, 28 + key_matrix_leds, 27 + key_matrix_leds
  };
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
    // led_count + 1, to account for the Neuron's LED. The last one is the
    // Neuron's LED, never send that to SLED.
    // clang-format off
  static constexpr uint8_t led_map[KeyboardLEDDriverProps::led_count] = {
        // left side - 35 keys includes LP
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34,

        // right side - 35 keys includes LP
        0 + LPH, 1 + LPH, 2 + LPH, 3 + LPH, 4 + LPH, 5 + LPH, 6 + LPH, 7 + LPH, 8 + LPH, 9 + LPH, 10 + LPH, 11 + LPH,
        12 + LPH, 13 + LPH, 14 + LPH,
        15 + LPH, 16 + LPH, 17 + LPH, 18 + LPH, 19 + LPH, 20 + LPH, 21 + LPH, 22 + LPH, 23 + LPH, 24 + LPH, 25 + LPH,
        26 + LPH, 27 + LPH, 28 + LPH,
        29 + LPH, 30 + LPH, 31 + LPH, 32 + LPH, 33 + LPH, 34 + LPH,

        // left under glow - 53
        35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61,
        62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87,

        // right underglow - 53
        35 + LPH, 36 + LPH, 37 + LPH, 38 + LPH, 39 + LPH, 40 + LPH, 41 + LPH, 42 + LPH, 43 + LPH, 44 + LPH, 45 + LPH,
        46 + LPH, 47 + LPH, 48 + LPH, 49 + LPH, 50 + LPH, 51 + LPH, 52 + LPH, 53 + LPH, 54 + LPH, 55 + LPH, 56 + LPH,
        57 + LPH,
        58 + LPH, 59 + LPH, 60 + LPH, 61 + LPH, 62 + LPH, 63 + LPH, 64 + LPH, 65 + LPH, 66 + LPH, 67 + LPH, 68 + LPH,
        69 + LPH, 70 + LPH, 71 + LPH, 72 + LPH, 73 + LPH, 74 + LPH, 75 + LPH, 76 + LPH, 77 + LPH, 78 + LPH, 79 + LPH,
        80 + LPH, 81 + LPH, 82 + LPH, 83 + LPH, 84 + LPH, 85 + LPH, 86 + LPH, 87 + LPH, 0xff
  };
    // clang-format on
};

struct KeyboardKeyScannerProps : public kaleidoscope::driver::keyscanner::BaseProps {
    static constexpr uint8_t matrix_rows    = 5;
    static constexpr uint8_t matrix_columns = 16;
    typedef MatrixAddr<matrix_rows, matrix_columns> KeyAddr;

    static constexpr uint8_t left_columns  = 8;
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
    static constexpr const char *short_name = "defy_wireless";
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
}  // namespace device

typedef kaleidoscope::device::dygma::KeyboardNrf Device;

}  // namespace kaleidoscope

#define PER_KEY_DATA(dflt,                                                                                  \
                     r0c0,                                                                                  \
                     r0c1,                                                                                  \
                     r0c2,                                                                                  \
                     r0c3,                                                                                  \
                     r0c4,                                                                                  \
                     r0c5,                                                                                  \
                     r0c6,                                                                                  \
                     r0c9,                                                                                  \
                     r0c10,                                                                                 \
                     r0c11,                                                                                 \
                     r0c12,                                                                                 \
                     r0c13,                                                                                 \
                     r0c14,                                                                                 \
                     r0c15,                                                                                 \
                     r1c0,                                                                                  \
                     r1c1,                                                                                  \
                     r1c2,                                                                                  \
                     r1c3,                                                                                  \
                     r1c4,                                                                                  \
                     r1c5,                                                                                  \
                     r1c6,                                                                                  \
                     r1c9,                                                                                  \
                     r1c10,                                                                                 \
                     r1c11,                                                                                 \
                     r1c12,                                                                                 \
                     r1c13,                                                                                 \
                     r1c14,                                                                                 \
                     r1c15,                                                                                 \
                     r2c0,                                                                                  \
                     r2c1,                                                                                  \
                     r2c2,                                                                                  \
                     r2c3,                                                                                  \
                     r2c4,                                                                                  \
                     r2c5,                                                                                  \
                     r2c6,                                                                                  \
                     r2c9,                                                                                  \
                     r2c10,                                                                                 \
                     r2c11,                                                                                 \
                     r2c12,                                                                                 \
                     r2c13,                                                                                 \
                     r2c14,                                                                                 \
                     r2c15,                                                                                 \
                     r3c0,                                                                                  \
                     r3c1,                                                                                  \
                     r3c2,                                                                                  \
                     r3c3,                                                                                  \
                     r3c4,                                                                                  \
                     r3c5,                                                                                  \
                     r3c10,                                                                                 \
                     r3c11,                                                                                 \
                     r3c12,                                                                                 \
                     r3c13,                                                                                 \
                     r3c14,                                                                                 \
                     r3c15,                                                                                 \
                     r4c0,                                                                                  \
                     r4c1,                                                                                  \
                     r4c2,                                                                                  \
                     r4c3,                                                                                  \
                     r4c12,                                                                                 \
                     r4c13,                                                                                 \
                     r4c14,                                                                                 \
                     r4c15,                                                                                 \
                     r4c7,                                                                                  \
                     r4c6,                                                                                  \
                     r4c5,                                                                                  \
                     r4c4,                                                                                  \
                     r4c8,                                                                                  \
                     r4c9,                                                                                  \
                     r4c10,                                                                                 \
                     r4c11)                                                                                 \
                                                                                                            \
    r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6, dflt, dflt, r0c9, r0c10, r0c11, r0c12, r0c13, r0c14, r0c15,   \
      r1c0, r1c1, r1c2, r1c3, r1c4, r1c5, r1c6, dflt, dflt, r1c9, r1c10, r1c11, r1c12, r1c13, r1c14, r1c15, \
      r2c0, r2c1, r2c2, r2c3, r2c4, r2c5, r2c6, dflt, dflt, r2c9, r2c10, r2c11, r2c12, r2c13, r2c14, r2c15, \
      r3c0, r3c1, r3c2, r3c3, r3c4, r3c5, dflt, dflt, dflt, dflt, r3c10, r3c11, r3c12, r3c13, r3c14, r3c15, \
      r4c0, r4c1, r4c2, r4c3, r4c4, r4c5, r4c6, r4c7, r4c8, r4c9, r4c10, r4c11, r4c12, r4c13, r4c14, r4c15


#define PER_KEY_DATA_STACKED(dflt,                                                                          \
                             r0c0,                                                                          \
                             r0c1,                                                                          \
                             r0c2,                                                                          \
                             r0c3,                                                                          \
                             r0c4,                                                                          \
                             r0c5,                                                                          \
                             r0c6,                                                                          \
                             r1c0,                                                                          \
                             r1c1,                                                                          \
                             r1c2,                                                                          \
                             r1c3,                                                                          \
                             r1c4,                                                                          \
                             r1c5,                                                                          \
                             r1c6,                                                                          \
                             r2c0,                                                                          \
                             r2c1,                                                                          \
                             r2c2,                                                                          \
                             r2c3,                                                                          \
                             r2c4,                                                                          \
                             r2c5,                                                                          \
                             r2c6,                                                                          \
                             r3c0,                                                                          \
                             r3c1,                                                                          \
                             r3c2,                                                                          \
                             r3c3,                                                                          \
                             r3c4,                                                                          \
                             r3c5,                                                                          \
                             r4c0,                                                                          \
                             r4c1,                                                                          \
                             r4c2,                                                                          \
                             r4c3,                                                                          \
                             r4c7,                                                                          \
                             r4c6,                                                                          \
                             r4c5,                                                                          \
                             r4c4,                                                                          \
                                                                                                            \
                             r0c9,                                                                          \
                             r0c10,                                                                         \
                             r0c11,                                                                         \
                             r0c12,                                                                         \
                             r0c13,                                                                         \
                             r0c14,                                                                         \
                             r0c15,                                                                         \
                             r1c9,                                                                          \
                             r1c10,                                                                         \
                             r1c11,                                                                         \
                             r1c12,                                                                         \
                             r1c13,                                                                         \
                             r1c14,                                                                         \
                             r1c15,                                                                         \
                             r2c9,                                                                          \
                             r2c10,                                                                         \
                             r2c11,                                                                         \
                             r2c12,                                                                         \
                             r2c13,                                                                         \
                             r2c14,                                                                         \
                             r2c15,                                                                         \
                             r3c10,                                                                         \
                             r3c11,                                                                         \
                             r3c12,                                                                         \
                             r3c13,                                                                         \
                             r3c14,                                                                         \
                             r3c15,                                                                         \
                             r4c12,                                                                         \
                             r4c13,                                                                         \
                             r4c14,                                                                         \
                             r4c15,                                                                         \
                             r4c8,                                                                          \
                             r4c9,                                                                          \
                             r4c10,                                                                         \
                             r4c11)                                                                         \
                                                                                                            \
    r0c0, r0c1, r0c2, r0c3, r0c4, r0c5, r0c6, dflt, dflt, r0c9, r0c10, r0c11, r0c12, r0c13, r0c14, r0c15,   \
      r1c0, r1c1, r1c2, r1c3, r1c4, r1c5, r1c6, dflt, dflt, r1c9, r1c10, r1c11, r1c12, r1c13, r1c14, r1c15, \
      r2c0, r2c1, r2c2, r2c3, r2c4, r2c5, r2c6, dflt, dflt, r2c9, r2c10, r2c11, r2c12, r2c13, r2c14, r2c15, \
      r3c0, r3c1, r3c2, r3c3, r3c4, r3c5, dflt, dflt, dflt, dflt, r3c10, r3c11, r3c12, r3c13, r3c14, r3c15, \
      r4c0, r4c1, r4c2, r4c3, r4c4, r4c5, r4c6, r4c7, r4c8, r4c9, r4c10, r4c11, r4c12, r4c13, r4c14, r4c15

#endif  // __DEFY_WIRELESS_H__
#endif
