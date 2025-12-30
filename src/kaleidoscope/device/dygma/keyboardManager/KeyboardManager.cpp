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
//#include <Kaleidoscope-LEDControl.h>

//#include "kaleidoscope/driver/color/GammaCorrection.h"
#include "kaleidoscope/driver/keyscanner/Base_Impl.h"
//#include "kaleidoscope/util/crc16.h"

//#include "common.h"
//
//#include "Twi_master.h"
//
//#include "Adafruit_USBD_Device.h"
//#include "Ble_manager.h"
#include "Communications.h"
#include "KeyboardManager.h"
//#include "Radio_manager.h"
#include "Status_leds.h"
#include "Wire.h" // Arduino Wire wrapper for the NRF52 chips
#include "universalModules/Focus.h"
#include "nrf_gpio.h"
//#include "Battery.h"

#include "LEDManager.h"


#define NEURON_LED_BRIGHTNESS 2

/* External prototypes */
extern bool_t kbd_glue_left_wired_connected( void );
extern bool_t kbd_glue_right_wired_connected( void );
extern void kbd_glue_side_power_left_set( bool_t power );
extern void kbd_glue_side_power_right_set( bool_t power );

//Twi_master twi_master(TWI_MASTER_SCL_PIN, TWI_MASTER_SDA_PIN);
Status_leds status_leds(LED_GREEN_PIN, LED_RED_PIN);


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

    static void setSidePower(bool power);
    static bool getSidePower()
    {
        return side_power_;
    }

    static void getChipID(char *buff, uint16_t len);
    static void get_chip_info(char *buff, uint16_t len);

  private:
    static bool side_power_;
    static uint16_t settings_interval_;
    static uint16_t settings_base;
};

dygma_keyboards::Hand KeyboardHands::leftHand(dygma_keyboards::Hand::LEFT);
dygma_keyboards::Hand KeyboardHands::rightHand(dygma_keyboards::Hand::RIGHT);
bool KeyboardHands::side_power_;
uint16_t KeyboardHands::settings_interval_;
uint16_t KeyboardHands::settings_base;

void KeyboardHands::setSidePower(bool power)
{
    // 0 -> reset keyboard side, 1 -> run keyboard side
    kbd_glue_side_power_left_set( power );
    kbd_glue_side_power_right_set( power );

    side_power_ = power;
}
// BLE       WIRED       RF
Communications_protocol::Devices leftConnection[3]{UNKNOWN, UNKNOWN, UNKNOWN};
Communications_protocol::Devices rightConnection[3]{UNKNOWN, UNKNOWN, UNKNOWN};

void KeyboardHands::setup()
{
    rightHand.init();
    leftHand.init();

    Communications.callbacks.bind(CONNECTED, (
                                                 [](const Packet &p)
                                                 {
                                                     if (p.header.device == BLE_DEFY_RIGHT) rightConnection[0] = BLE_DEFY_RIGHT;
                                                     if (p.header.device == BLE_DEFY_LEFT) leftConnection[0] = BLE_DEFY_LEFT;
                                                     if (p.header.device == KEYSCANNER_DEFY_LEFT)
                                                         leftConnection[1] = ble_innited() ? BLE_DEFY_LEFT : KEYSCANNER_DEFY_LEFT;
                                                     if (p.header.device == KEYSCANNER_DEFY_RIGHT)
                                                         rightConnection[1] = ble_innited() ? BLE_DEFY_RIGHT : KEYSCANNER_DEFY_RIGHT;
                                                     if (p.header.device == RF_DEFY_LEFT) leftConnection[2] = RF_DEFY_LEFT;
                                                     if (p.header.device == RF_DEFY_RIGHT) rightConnection[2] = RF_DEFY_RIGHT;

                                                     auto &keyScanner = Runtime.device().keyScanner();
                                                     auto isKSLeftWired = keyScanner.leftSideWiredConnection();
                                                     auto isKSRightWired = keyScanner.rightSideWiredConnection();
                                                     LEDManager.com_mode_set( isKSLeftWired && isKSRightWired && !ble_innited() );
                                                     LEDManager.leds_enable();
                                                 }));
    Communications.callbacks.bind(DISCONNECTED, (
                                                    [](const Packet &p)
                                                    {
                                                        if (p.header.device == BLE_DEFY_RIGHT)
                                                        {
                                                            rightConnection[0] = UNKNOWN;
                                                            rightConnection[1] = UNKNOWN;
                                                        }
                                                        if (p.header.device == BLE_DEFY_LEFT)
                                                        {
                                                            leftConnection[0] = UNKNOWN;
                                                            leftConnection[1] = UNKNOWN;
                                                        }
                                                        if (p.header.device == KEYSCANNER_DEFY_LEFT) leftConnection[1] = UNKNOWN;
                                                        if (p.header.device == KEYSCANNER_DEFY_RIGHT) rightConnection[1] = UNKNOWN;
                                                        if (p.header.device == RF_DEFY_LEFT) leftConnection[2] = UNKNOWN;
                                                        if (p.header.device == RF_DEFY_RIGHT) rightConnection[2] = UNKNOWN;

                                                        if ( leftConnection[0] == UNKNOWN &&
                                                                leftConnection[1] == UNKNOWN &&
                                                                leftConnection[2] == UNKNOWN )
                                                        {
                                                            leftHand.releaseAllKeys();
                                                        }

                                                        if ( rightConnection[0] == UNKNOWN &&
                                                                rightConnection[1] == UNKNOWN &&
                                                                rightConnection[2] == UNKNOWN )
                                                        {
                                                            rightHand.releaseAllKeys();
                                                        }
                                                    }));
}

void KeyboardHands::getChipID(char *cstring, uint16_t len)
{
    /*
        Returns the 64 bit unique device identifier.

        See: FICR - Factory information configuration registers on pag. 30 of the datasheet.

        returns a cstring.
    */

    snprintf(cstring, len, "%8lx%8lx", NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0]);
}

void KeyboardHands::get_chip_info(char *cstring, uint16_t len)
{
    /*
        See: FICR - Factory information configuration registers on pag. 30 of the datasheet.

        returns a cstring.
    */

    snprintf(cstring, len, "DEVICEID=%8lx%8lx\nPART=%lx\nVARIANT=%lx\nPACKAGE=%lx\nRAM=%ld\nFLASH=%ld", NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0],
             NRF_FICR->INFO.PART, NRF_FICR->INFO.VARIANT, NRF_FICR->INFO.PACKAGE, NRF_FICR->INFO.RAM, NRF_FICR->INFO.FLASH);
}

/********* Key scanner *********/

dygma_keyboards::key_data KeyboardKeyScanner::leftHandState;
dygma_keyboards::key_data KeyboardKeyScanner::rightHandState;
dygma_keyboards::key_data KeyboardKeyScanner::previousLeftHandState;
dygma_keyboards::key_data KeyboardKeyScanner::previousRightHandState;
dygma_keyboards::key_data KeyboardKeyScanner::leftHandMask;
dygma_keyboards::key_data KeyboardKeyScanner::rightHandMask;

// Extended format for keyboards with >8 columns
dygma_keyboards::key_data_extended KeyboardKeyScanner::leftHandStateExtended;
dygma_keyboards::key_data_extended KeyboardKeyScanner::rightHandStateExtended;
dygma_keyboards::key_data_extended KeyboardKeyScanner::previousLeftHandStateExtended;
dygma_keyboards::key_data_extended KeyboardKeyScanner::previousRightHandStateExtended;

void KeyboardKeyScanner::scanMatrix()
{
   // usbConnectionsStateMachine();
    readMatrix();
    actOnMatrixScan();
}

void KeyboardKeyScanner::readMatrix()
{
    previousLeftHandState = leftHandState;
    previousRightHandState = rightHandState;
    previousLeftHandStateExtended = leftHandStateExtended;
    previousRightHandStateExtended = rightHandStateExtended;

    if (KeyboardHands::leftHand.newKey())
    {
        leftHandState = KeyboardHands::leftHand.getKeyData();
        // Store extended data if available (for Sonshi with 12 columns)
        if (KeyboardHands::leftHand.isExtendedFormat()) {
            leftHandStateExtended = KeyboardHands::leftHand.getKeyDataExtended();
        }
    }
    if (KeyboardHands::rightHand.newKey())
    {
        rightHandState = KeyboardHands::rightHand.getKeyData();
        // Store extended data if available
        if (KeyboardHands::rightHand.isExtendedFormat()) {
            rightHandStateExtended = KeyboardHands::rightHand.getKeyDataExtended();
        }
    }
}

void KeyboardKeyScanner::actOnMatrixScan()
{
    // Check if we're using extended format (for keyboards with >8 columns like Sonshi)
    bool useExtendedLeft = KeyboardHands::leftHand.isExtendedFormat();
    bool useExtendedRight = KeyboardHands::rightHand.isExtendedFormat();
    
    for (uint8_t row = 0; row < Props_::matrix_rows; row++)
    {
        for (uint8_t col = 0; col < Props_::left_columns; col++)
        {
            uint8_t keyState;

            // left - use extended format if available, otherwise standard
            bool prevState, currState;
            if (useExtendedLeft) {
                prevState = (previousLeftHandStateExtended.rows[row] & (1 << col)) != 0;
                currState = (leftHandStateExtended.rows[row] & (1 << col)) != 0;
            } else {
                prevState = (previousLeftHandState.rows[row] & (1 << col)) != 0;
                currState = (leftHandState.rows[row] & (1 << col)) != 0;
            }
            keyState = (prevState << 0) | (currState << 1);

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

            // right - use extended format if available, otherwise standard
            if (useExtendedRight) {
                prevState = (previousRightHandStateExtended.rows[row] & (1 << col)) != 0;
                currState = (rightHandStateExtended.rows[row] & (1 << col)) != 0;
            } else {
                prevState = (previousRightHandState.rows[row] & (1 << col)) != 0;
                currState = (rightHandState.rows[row] & (1 << col)) != 0;
            }
            keyState = (prevState << 0) | (currState << 1);

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
    uint8_t count = 0;
    for (int i = 0; i < 5; i++) {
        count += __builtin_popcount(leftHandState.rows[i]);
        count += __builtin_popcount(rightHandState.rows[i]);
    }
    return count;
}

uint8_t KeyboardKeyScanner::previousPressedKeyswitchCount()
{
    uint8_t count = 0;
    for (int i = 0; i < 5; i++) {
        count += __builtin_popcount(previousLeftHandState.rows[i]);
        count += __builtin_popcount(previousRightHandState.rows[i]);
    }
    return count;
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
    memset(&leftHandState, 0, sizeof(leftHandState));
    memset(&rightHandState, 0, sizeof(rightHandState));
    Runtime.hid().keyboard().releaseAllKeys();
    Runtime.hid().keyboard().sendReport();
}

Communications_protocol::Devices KeyboardKeyScanner::leftHandDevice(void)
{
    for (const auto &connection : leftConnection)
    {
        if (connection != UNKNOWN)
        {
            return connection;
        }
    }

    return UNKNOWN;
}

Communications_protocol::Devices KeyboardKeyScanner::rightHandDevice(void)
{
    for (const auto &connection : rightConnection)
    {
        if (connection != UNKNOWN)
        {
            return connection;
        }
    }

    return UNKNOWN;
}

bool KeyboardKeyScanner::rightSideWiredConnection()
{
    return kbd_glue_right_wired_connected();
}

bool KeyboardKeyScanner::leftSideWiredConnection()
{
    return kbd_glue_left_wired_connected();
}

/********* KeyboardNrf class (Hardware plugin) *********/

void KeyboardNrf::setup()
{
    // Check if we can live without this reset sides
    kbd_glue_side_power_left_set( true );
    kbd_glue_side_power_right_set( true );

    status_leds.init();
    status_leds.static_green(NEURON_LED_BRIGHTNESS);

    KeyboardHands::setup();
    KeyboardFocus.init();
    KeyScanner::setup();
}

uint8_t KeyboardNrf::side::getPower()
{
    return KeyboardHands::getSidePower();
}

void KeyboardNrf::side::setPower(uint8_t power)
{
    KeyboardHands::setSidePower(power);
}

uint8_t KeyboardNrf::side::leftVersion()
{
    // TODO: Versions of keyscanner
    return 0;
    //  return KeyboardHands::hand_spi1.readVersion();
}

uint8_t KeyboardNrf::side::rightVersion()
{
    // TODO: Versions of keyscanner
    return 0;

    //  return KeyboardHands::hand_spi2.readVersion();
}

#warning "Resolve this"
//void KeyboardNrf::side::reset_sides()
//{
//    nrf_gpio_cfg_output(SIDE_NRESET_1);
//    nrf_gpio_cfg_output(SIDE_NRESET_2);
//    nrf_gpio_pin_write(SIDE_NRESET_1, 0);
//    nrf_gpio_pin_write(SIDE_NRESET_2, 0);
//    delay(10);
//    nrf_gpio_cfg_input(SIDE_NRESET_1, NRF_GPIO_PIN_NOPULL);
//    nrf_gpio_cfg_input(SIDE_NRESET_2, NRF_GPIO_PIN_NOPULL);
//    delay(50); // We should give a bit more time but for now lest leave it like this
//}
//
//void KeyboardNrf::side::reset_right_side()
//{
//    nrf_gpio_cfg_output(SIDE_NRESET_1);
//    nrf_gpio_pin_write(SIDE_NRESET_1, 0);
//    delay(10);
//    nrf_gpio_cfg_input(SIDE_NRESET_1, NRF_GPIO_PIN_NOPULL);
//    delay(50); // We should give a bit more time but for now lest leave it like this
//}
//
//void KeyboardNrf::side::reset_left_side()
//{
//    nrf_gpio_cfg_output(SIDE_NRESET_2);
//    nrf_gpio_pin_write(SIDE_NRESET_2, 0);
//    delay(10);
//    nrf_gpio_cfg_input(SIDE_NRESET_2, NRF_GPIO_PIN_NOPULL);
//    delay(50); // We should give a bit more time but for now lest leave it like this
//}

void KeyboardNrf::side::prepareForFlash()
{
    Wire::begin(100);
}

void KeyboardNrf::settings::getChipID(char *buff, uint16_t len)
{
    KeyboardHands::getChipID(buff, len);
}

void KeyboardNrf::settings::get_chip_info(char *buff, uint16_t len)
{
    KeyboardHands::get_chip_info(buff, len);
}

} // namespace dygma
} // namespace device
} // namespace kaleidoscope


#endif  // ARDUINO_ARCH_NRF52
