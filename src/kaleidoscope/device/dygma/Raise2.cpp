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


#include "kaleidoscope/Runtime.h"
#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-LEDControl.h>

#include "kaleidoscope/driver/color/GammaCorrection.h"
#include "kaleidoscope/driver/keyscanner/Base_Impl.h"
#include "kaleidoscope/util/crc16.h"

#include "common.h"

#include "Twi_master.h"

#include "Adafruit_USBD_Device.h"
#include "Ble_manager.h"
#include "Colormap-Defy.h"
#include "Communications.h"
#include "Raise2.h"
#include "LED-Palette-Theme-Defy.h"
#include "Radio_manager.h"
#include "Status_leds.h"
#include "Wire.h" // Arduino Wire wrapper for the NRF52 chips
#include "raise2/Focus.h"
#include "nrf_gpio.h"
#include "Battery.h"


#define NEURON_LED_BRIGHTNESS 2


Twi_master twi_master(TWI_MASTER_SCL_PIN, TWI_MASTER_SDA_PIN);
Status_leds status_leds(LED_GREEN_PIN, LED_RED_PIN);


namespace kaleidoscope
{
namespace device
{
namespace dygma
{

/********* Raise2Hands *********/

struct Raise2Hands
{
    static raise2::Hand leftHand;
    static raise2::Hand rightHand;

    static void setup();

    static void setSidePower(bool power);
    static bool getSidePower()
    {
        return side_power_;
    }

    static void keyscanInterval(uint8_t interval);
    static uint8_t keyscanInterval()
    {
        return keyscan_interval_;
    }

    static void getChipID(char *buff, uint16_t len);
    static void get_chip_info(char *buff, uint16_t len);

    static void ledBrightnessLedDriver(uint8_t brightness);
    static uint8_t ledBrightnessLedDriver()
    {
        return bright.led_brightness_ledDriver_;
    }

    static void ledBrightnessUG(uint8_t brightnessUG);
    static uint8_t ledBrightnessUG()
    {
        return bright.led_brightness_underglow_;
    }

    static void ledBrightnessLedDriverWireless(uint8_t brightness);
    static uint8_t ledBrightnessLedDriverWireless()
    {
        return bright.led_brightness_ledDriver_wireless_;
    }

    static void ledBrightnessUGWireless(uint8_t brightnessUG);
    static uint8_t ledBrightnessUGWireless()
    {
        return bright.led_brightness_underglow_wireless_;
    }
    static struct Brightness
    {
        uint8_t led_brightness_ledDriver_;
        uint8_t led_brightness_underglow_;
        uint8_t led_brightness_ledDriver_wireless_;
        uint8_t led_brightness_underglow_wireless_;
        uint8_t flag;
    } bright;

    static void sendPacketBrightness();

  private:
    static uint8_t keyscan_interval_;
    static bool side_power_;
    static uint16_t settings_interval_;
    static uint16_t settings_base;

    static void setbrightness(const Brightness &data);
};

raise2::Hand Raise2Hands::leftHand(raise2::Hand::LEFT);
raise2::Hand Raise2Hands::rightHand(raise2::Hand::RIGHT);
bool Raise2Hands::side_power_;
uint16_t Raise2Hands::settings_interval_;
uint16_t Raise2Hands::settings_base;
Raise2Hands::Brightness Raise2Hands::bright;
uint8_t Raise2Hands::keyscan_interval_ = 15;

void Raise2Hands::setSidePower(bool power)
{
    // 0 -> reset keyboard side, 1 -> run keyboard side
    if (power)
    {
        nrf_gpio_cfg_input(SIDE_NRESET_1, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_input(SIDE_NRESET_2, NRF_GPIO_PIN_NOPULL);
    }
    else
    {
        nrf_gpio_cfg_output(SIDE_NRESET_1);
        nrf_gpio_cfg_output(SIDE_NRESET_2);
        nrf_gpio_pin_write(SIDE_NRESET_1, power);
        nrf_gpio_pin_write(SIDE_NRESET_2, power);
    }


    side_power_ = power;
}
// BLE       WIRED       RF
Communications_protocol::Devices leftConnection[3]{UNKNOWN, UNKNOWN, UNKNOWN};
Communications_protocol::Devices rightConnection[3]{UNKNOWN, UNKNOWN, UNKNOWN};

auto checkBrightness = [](const Packet &)
{
    if (!::LEDControl.isEnabled())
    {
        status_leds.stop_all();

        Communications_protocol::Packet p{};
        p.header.command = Communications_protocol::BRIGHTNESS;
        p.header.device = UNKNOWN;
        p.data[0] = 0;
        p.data[1] = 0;
        p.data[2] = static_cast<uint8_t>(ColormapEffectDefy.no_led_effect);
        p.data[3] = 1;
        p.header.size = 4;
        Communications.sendPacket(p);

        return;
    }

    status_leds.static_green(NEURON_LED_BRIGHTNESS);
    auto &keyScanner = Runtime.device().keyScanner();
    auto isRaise2LeftWired = keyScanner.leftSideWiredConnection();
    auto isRaise2RightWired = keyScanner.rightSideWiredConnection();

    if(isRaise2LeftWired && isRaise2RightWired)
    {
            ColormapEffectDefy.updateBrigthness(ColormapEffectDefy.no_led_effect, true, true);
    }
    else
    {
            ColormapEffectDefy.updateBrigthness(ColormapEffectDefy.no_led_effect, true, false);
    }
};

void Raise2Hands::setup()
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
                                                     ::LEDControl.enable();
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
                                                    }));

    Communications.callbacks.bind(DISCONNECTED, checkBrightness);
    Communications.callbacks.bind(CONNECTED, checkBrightness);
    Communications.callbacks.bind(CONNECTED, ([](const Packet &) { ::LEDControl.set_mode(::LEDControl.get_mode_index()); }));


    settings_interval_ = ::EEPROMSettings.requestSlice(sizeof(keyscan_interval_));
    settings_base = ::EEPROMSettings.requestSlice(sizeof(Raise2Hands::Brightness));
    // If keyscan is max, assume that EEPROM is uninitialized, and store the defaults.
    uint16_t interval;
    Runtime.storage().get(settings_interval_, interval);
    if (interval == 0xff)
    {
        Runtime.storage().put(settings_interval_, keyscan_interval_);
        Runtime.storage().commit();
    }
    Runtime.storage().get(settings_interval_, keyscan_interval_);

    Raise2Hands::Brightness brightness;
    Runtime.storage().get(settings_base, brightness);
    if (brightness.flag != 0)
    {
        bright.led_brightness_ledDriver_ = 255;
        bright.led_brightness_underglow_ = 255;
        bright.led_brightness_underglow_wireless_ = 125;
        bright.led_brightness_ledDriver_wireless_ = 125;
        bright.flag = 0;
        setbrightness(bright);
    }
    Runtime.storage().get(settings_base, bright);
}

void Raise2Hands::setbrightness(const Brightness &data)
{
    Runtime.storage().put(settings_base, data);
    Runtime.storage().commit();
}

void Raise2Hands::keyscanInterval(uint8_t interval)
{
    Communications_protocol::Packet p{};
    p.header.command = Communications_protocol::KEYSCAN_INTERVAL;
    p.data[0] = interval;
    p.header.size = 1;
    Communications.sendPacket(p);
    keyscan_interval_ = interval;
    Runtime.storage().put(settings_interval_, keyscan_interval_);
    Runtime.storage().commit();
}

void Raise2Hands::ledBrightnessLedDriver(uint8_t brightness)
{
    bright.led_brightness_ledDriver_ = brightness;
    sendPacketBrightness();
    setbrightness(bright);
}

void Raise2Hands::ledBrightnessUG(uint8_t brightnessUG)
{
    bright.led_brightness_underglow_ = brightnessUG;
    sendPacketBrightness();
    setbrightness(bright);
}

void Raise2Hands::ledBrightnessLedDriverWireless(uint8_t brightness)
{
    bright.led_brightness_ledDriver_wireless_ = brightness;
    sendPacketBrightness();
    setbrightness(bright);
}
void Raise2Hands::ledBrightnessUGWireless(uint8_t brightnessUG)
{
    bright.led_brightness_underglow_wireless_ = brightnessUG;
    sendPacketBrightness();
    setbrightness(bright);
}

void Raise2Hands::sendPacketBrightness()
{
    Packet p{};
    checkBrightness(p);
}

void Raise2Hands::getChipID(char *cstring, uint16_t len)
{
    /*
        Returns the 64 bit unique device identifier.

        See: FICR - Factory information configuration registers on pag. 30 of the datasheet.

        returns a cstring.
    */

    snprintf(cstring, len, "%8lx%8lx", NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0]);
}

void Raise2Hands::get_chip_info(char *cstring, uint16_t len)
{
    /*
        See: FICR - Factory information configuration registers on pag. 30 of the datasheet.

        returns a cstring.
    */

    snprintf(cstring, len, "DEVICEID=%8lx%8lx\nPART=%lx\nVARIANT=%lx\nPACKAGE=%lx\nRAM=%ld\nFLASH=%ld", NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0],
             NRF_FICR->INFO.PART, NRF_FICR->INFO.VARIANT, NRF_FICR->INFO.PACKAGE, NRF_FICR->INFO.RAM, NRF_FICR->INFO.FLASH);
}


/********* LED Driver *********/

bool Raise2LEDDriver::isLEDChangedNeuron;
bool Raise2LEDDriver::leds_enabled_ = true;
uint8_t Raise2LEDDriver::isLEDChangedLeft[LED_BANKS];
uint8_t Raise2LEDDriver::isLEDChangedRight[LED_BANKS];
cRGB Raise2LEDDriver::neuronLED;
// Add this line to define the static member variable
constexpr uint8_t Raise2LEDDriver::led_map[Raise2LEDDriverProps::led_count];
//constexpr uint8_t Raise2LEDDriver::led_map[Raise2LEDDriverProps::led_count];
constexpr uint8_t Raise2LEDDriverProps::key_led_map[];

// Wired setters and getters
void Raise2LEDDriver::setBrightness(uint8_t brightness)
{
    Raise2Hands::ledBrightnessLedDriver(brightness);
}

uint8_t Raise2LEDDriver::getBrightness()
{
    return Raise2Hands::ledBrightnessLedDriver();
}

void Raise2LEDDriver::setBrightnessUG(uint8_t brightnessUG)
{
    Raise2Hands::ledBrightnessUG(brightnessUG);
}

uint8_t Raise2LEDDriver::getBrightnessUG()
{
    return Raise2Hands::ledBrightnessUG();
}
// Wireless setters and getters
void Raise2LEDDriver::setBrightnessWireless(uint8_t brightness)
{
    Raise2Hands::ledBrightnessLedDriverWireless(brightness);
}

uint8_t Raise2LEDDriver::getBrightnessWireless()
{
    return Raise2Hands::ledBrightnessLedDriverWireless();
}

void Raise2LEDDriver::setBrightnessUGWireless(uint8_t brightnessUG)
{
    Raise2Hands::ledBrightnessUGWireless(brightnessUG);
}

uint8_t Raise2LEDDriver::getBrightnessUGWireless()
{
    return Raise2Hands::ledBrightnessUGWireless();
}

void Raise2LEDDriver::syncLeds()
{
    bool is_enabled = ::LEDControl.isEnabled();

    if (leds_enabled_ != is_enabled)
    {
        leds_enabled_ = is_enabled;
        Raise2Hands::sendPacketBrightness();
    }

    if (isLEDChangedNeuron)
    {
        updateNeuronLED();
        isLEDChangedNeuron = false;
    }

    if (isLEDChangedNeuron)
    {
        updateNeuronLED();
        isLEDChangedNeuron = false;
    }
}

void Raise2LEDDriver::updateNeuronLED()
{
    // static constexpr struct
    // {
    //   uint8_t r, g, b;
    // } pins = {3, 5, 4};

    // invert as these are common anode, and make sure we reach 65535 to be able
    // to turn fully off.
    /*analogWrite(pins.r, ((256 - pgm_read_byte(&gamma8[neuronLED.r])) << 8) - 1);
  analogWrite(pins.g, ((256 - pgm_read_byte(&gamma8[neuronLED.g])) << 8) - 1);
  analogWrite(pins.b, ((256 - pgm_read_byte(&gamma8[neuronLED.b])) << 8) - 1);*/
}

void Raise2LEDDriver::setCrgbAt(uint8_t i, cRGB crgb)
{
    // prevent reading off the end of the led_map array
    if (i >= Raise2LEDDriverProps::led_count) return;

    // neuron LED
    if (i == Raise2LEDDriverProps::led_count - 2)
    {
        isLEDChangedNeuron |= !(neuronLED.r == crgb.r && neuronLED.g == crgb.g && neuronLED.b == crgb.b && neuronLED.w == crgb.w);
        neuronLED = crgb;
        return;
    }

    // get the SLED index
    uint8_t sled_num = Raise2LEDDriver::led_map[i];
    if (sled_num < LEDS_PER_HAND)
    {
        cRGB oldColor = Raise2Hands::leftHand.led_data.leds[sled_num];
        Raise2Hands::leftHand.led_data.leds[sled_num] = crgb;
        isLEDChangedLeft[uint8_t(sled_num / 8)] |= !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b && oldColor.w == crgb.w);
    }
    else if (sled_num < 2 * LEDS_PER_HAND)
    {
        cRGB oldColor = Raise2Hands::rightHand.led_data.leds[sled_num - LEDS_PER_HAND];
        Raise2Hands::rightHand.led_data.leds[sled_num - LEDS_PER_HAND] = crgb;
        isLEDChangedRight[uint8_t((sled_num - LEDS_PER_HAND) / 8)] |=
            !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b && oldColor.w == crgb.w);
    }
}

// void WiredLEDDriver::setCrgbNeuron(cRGB crgb) {
//   isLEDChangedNeuron |= !(neuronLED.r==crgb.r && neuronLED.g==crgb.g && neuronLED.b==crgb.b && neuronLED.w==crgb.w);
//   neuronLED = crgb;
// }

cRGB Raise2LEDDriver::getCrgbAt(uint8_t i)
{
    if (i >= Raise2LEDDriverProps::led_count) return {0, 0, 0};

    uint8_t sled_num = Raise2LEDDriver::led_map[i];
    if (sled_num < LEDS_PER_HAND)
    {
        return Raise2Hands::leftHand.led_data.leds[sled_num];
    }
    else if (sled_num < 2 * LEDS_PER_HAND)
    {
        return Raise2Hands::rightHand.led_data.leds[sled_num - LEDS_PER_HAND];
    }
    else
    {
        return {0, 0, 0};
    }
}

void Raise2LEDDriver::setup()
{
    // arduino zero analogWrite(255) isn't fully on as its actually working with a
    // 16bit counter and the mapping is a bit shift.
    // so change to 16 bit resolution to avoid the mapping and do the mapping
    // ourselves in updateHubleLED() to ensure LEDs can be set fully off
    // analogWriteResolution(16);
    updateNeuronLED();
}

/********* Key scanner *********/

raise2::key_data Raise2KeyScanner::leftHandState;
raise2::key_data Raise2KeyScanner::rightHandState;
raise2::key_data Raise2KeyScanner::previousLeftHandState;
raise2::key_data Raise2KeyScanner::previousRightHandState;
raise2::key_data Raise2KeyScanner::leftHandMask;
raise2::key_data Raise2KeyScanner::rightHandMask;

void Raise2KeyScanner::scanMatrix()
{
    usbConnectionsStateMachine();
    readMatrix();
    actOnMatrixScan();
}

void Raise2KeyScanner::readMatrix()
{
    previousLeftHandState = leftHandState;
    previousRightHandState = rightHandState;

    if (Raise2Hands::leftHand.newKey())
    {
        leftHandState = Raise2Hands::leftHand.getKeyData();
    }
    if (Raise2Hands::rightHand.newKey())
    {
        rightHandState = Raise2Hands::rightHand.getKeyData();
    }
}

void Raise2KeyScanner::actOnMatrixScan()
{
    for (uint8_t row = 0; row < Props_::matrix_rows; row++)
    {
        for (uint8_t col = 0; col < Props_::left_columns; col++)
        {
            uint8_t keynum = (row * Props_::left_columns) + col;
            uint8_t keyState;

            // left
            keyState = (bitRead(previousLeftHandState.all, keynum) << 0) | (bitRead(leftHandState.all, keynum) << 1);

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
            keyState = (bitRead(previousRightHandState.all, keynum) << 0) | (bitRead(rightHandState.all, keynum) << 1);

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

void Raise2KeyScanner::maskKey(KeyAddr key_addr)
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

void Raise2KeyScanner::unMaskKey(KeyAddr key_addr)
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

bool Raise2KeyScanner::isKeyMasked(KeyAddr key_addr)
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

void Raise2KeyScanner::maskHeldKeys()
{
    memcpy(leftHandMask.rows, leftHandState.rows, sizeof(leftHandMask));
    memcpy(rightHandMask.rows, rightHandState.rows, sizeof(rightHandMask));
}

bool Raise2KeyScanner::isKeyswitchPressed(KeyAddr key_addr)
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

bool Raise2KeyScanner::wasKeyswitchPressed(KeyAddr key_addr)
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

uint8_t Raise2KeyScanner::pressedKeyswitchCount()
{
    return __builtin_popcountll(leftHandState.all) + __builtin_popcountll(rightHandState.all);
}

uint8_t Raise2KeyScanner::previousPressedKeyswitchCount()
{
    return __builtin_popcountll(previousLeftHandState.all) + __builtin_popcountll(previousRightHandState.all);
}

void Raise2KeyScanner::setup()
{
    static constexpr uint8_t keyscanner_pins[] = {2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                                                  21, 22, 23, 24, 25, 26, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42};

    for (uint32_t i = 0; i < sizeof(keyscanner_pins); i++)
    {
        // pinMode(keyscanner_pins[i], OUTPUT);
        // digitalWrite(keyscanner_pins[i], LOW);
    }
}

void Raise2KeyScanner::reset(void)
{
    leftHandState.all = 0;
    rightHandState.all = 0;
    Runtime.hid().keyboard().releaseAllKeys();
    Runtime.hid().keyboard().sendReport();
}

Communications_protocol::Devices Raise2KeyScanner::leftHandDevice(void)
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

Communications_protocol::Devices Raise2KeyScanner::rightHandDevice(void)
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

void Raise2KeyScanner::usbConnectionsStateMachine()
{
    uint32_t actualTime = millis();
    bool usbMounted = TinyUSBDevice.mounted();
    bool bleInitiated = ble_innited();
    bool radioInited = kaleidoscope::plugin::RadioManager::isInited();
    bool forceBle = _BleManager.getForceBle();
    static bool flag_ble_mode_allowed = true;

    uint8_t bat_status_l = kaleidoscope::plugin::Battery::get_battery_status_left();
    uint8_t bat_status_r = kaleidoscope::plugin::Battery::get_battery_status_right();
    /*
        0 -> Side connected and powered from its battery or the other side's battery.
        1 o 2 -> Side connected and powered from the N2 while it is connected to the PC via USB.
        4 -> Side disconnected.
    */
    if ( (bat_status_l == 1 || bat_status_l == 2 || bat_status_r == 1 || bat_status_r == 2) &&
        flag_ble_mode_allowed)
    {
        flag_ble_mode_allowed = false;
        NRF_LOG_DEBUG("BLE mode denied due to external power on N2.");
    }

    // For 3000ms at the 3100ms mark, check whether to initialize BLE or RF
    if ( actualTime > 3000 && actualTime < 3100 &&
        !bleInitiated &&
        !radioInited )
    {
        if (usbMounted && !forceBle)
        {
            kaleidoscope::plugin::RadioManager::init();
        }
        else
        {
            if (flag_ble_mode_allowed)
            {
                //Force connnect again just in case it was set as a device and not a host
                _BleManager.init();

                if (leftConnection[1] == KEYSCANNER_DEFY_LEFT)
                {
                    leftConnection[1] = BLE_DEFY_LEFT;
                }

                if (rightConnection[1] == KEYSCANNER_DEFY_RIGHT)
                {
                    rightConnection[1] = BLE_DEFY_RIGHT;
                }

                Raise2Hands::sendPacketBrightness();

                _BleManager.setForceBle(false);

                Packet p{};
                p.header.command = CONNECTED;
                p.header.size = 0;
                p.header.device = BLE_NEURON_2_DEFY;
                Communications.sendPacket(p);
            }
        }
    }

    //Only in the case that we have ble init and there is not any usb connected we reboot the system
    if( actualTime > 4000 && ble_innited() && !nrf_gpio_pin_read(SIDE_NRESET_1) && !nrf_gpio_pin_read(SIDE_NRESET_2) )
    {
        reset_mcu();
    }
}

bool Raise2KeyScanner::rightSideWiredConnection()
{
    return nrf_gpio_pin_read(SIDE_NRESET_2);
}

bool Raise2KeyScanner::leftSideWiredConnection()
{
    return nrf_gpio_pin_read(SIDE_NRESET_1);
}

/********* Raise2 class (Hardware plugin) *********/

void Raise2::setup()
{
    // Check if we can live without this reset sides
    nrf_gpio_cfg_input(SIDE_NRESET_1, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(SIDE_NRESET_2, NRF_GPIO_PIN_NOPULL);

    status_leds.init();
    status_leds.static_green(NEURON_LED_BRIGHTNESS);

    Raise2Hands::setup();
    Raise2Focus.init();
    KeyScanner::setup();
    LEDDriver::setup();
}

void Raise2LEDDriver::setCrgbNeuron(cRGB crgb)
{
    isLEDChangedNeuron |= !(neuronLED.r == crgb.r && neuronLED.g == crgb.g && neuronLED.b == crgb.b && neuronLED.w == crgb.w);
    neuronLED = crgb;
}

uint8_t Raise2::side::getPower()
{
    return Raise2Hands::getSidePower();
}

void Raise2::side::setPower(uint8_t power)
{
    Raise2Hands::setSidePower(power);
}

uint8_t Raise2::side::leftVersion()
{
    // TODO: Versions of keyscanner
    return 0;
    //  return Raise2Hands::hand_spi1.readVersion();
}

uint8_t Raise2::side::rightVersion()
{
    // TODO: Versions of keyscanner
    return 0;

    //  return Raise2Hands::hand_spi2.readVersion();
}

void Raise2::side::reset_sides()
{
    nrf_gpio_cfg_output(SIDE_NRESET_1);
    nrf_gpio_cfg_output(SIDE_NRESET_2);
    nrf_gpio_pin_write(SIDE_NRESET_1, 0);
    nrf_gpio_pin_write(SIDE_NRESET_2, 0);
    delay(10);
    nrf_gpio_cfg_input(SIDE_NRESET_1, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(SIDE_NRESET_2, NRF_GPIO_PIN_NOPULL);
    delay(10); // We should give a bit more time but for now lest leave it like this
}

void Raise2::side::prepareForFlash()
{
    Wire::begin(100);
}

uint16_t Raise2::settings::keyscanInterval()
{
    return Raise2Hands::keyscanInterval();
}

void Raise2::settings::getChipID(char *buff, uint16_t len)
{
    Raise2Hands::getChipID(buff, len);
}

void Raise2::settings::get_chip_info(char *buff, uint16_t len)
{
    Raise2Hands::get_chip_info(buff, len);
}

void Raise2::settings::keyscanInterval(uint16_t interval)
{
    Raise2Hands::keyscanInterval(interval);
}

} // namespace dygma
} // namespace device
} // namespace kaleidoscope


#endif  // ARDUINO_ARCH_NRF52
