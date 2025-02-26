/* -*- mode: c++ -*-
* kaleidoscope::device::dygma::Wired -- Kaleidoscope device plugin for Dygma Wired
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

#ifdef ARDUINO_RASPBERRY_PI_PICO

#include "kaleidoscope/Runtime.h"
#include <Kaleidoscope-EEPROM-Settings.h>
#include <Wire.h>
#include "pico/unique_id.h"

#include "kaleidoscope/util/crc16.h"
#include "kaleidoscope/driver/color/GammaCorrection.h"
#include "kaleidoscope/driver/keyscanner/Base_Impl.h"
#include "DefyWN.h"
#include "Colormap-Defy.h"
#include "Communications_protocol.h"
#include "kaleidoscope/device/dygma/defyWN/universalModules/Hand.h"

#include "common.h"

namespace kaleidoscope {
namespace device {
namespace dygma {

class Hands {
public:
 inline static defyWN::Hand leftHand{Communications_protocol::Devices::KEYSCANNER_DEFY_LEFT};
 inline static defyWN::Hand rightHand{Communications_protocol::Devices::KEYSCANNER_DEFY_RIGHT};

 static void setup();

 static void setKeyscanInterval(uint8_t interval);
 static uint8_t getKeyscanInterval() { return settings_.keyscan_interval; }

 static void setLedBrightnessLedDriver(uint8_t brightness);
 static uint8_t getLedBrightnessLedDriver() {
   return settings_.led_brightness_ledDriver;
 }
 static void setLedBrightnessUG(uint8_t brightness);
 static uint8_t getLedBrightnessUG() {
   return settings_.led_brightness_ungerlow;
 }

 static void sendPacketBrightness();

private:
 struct Settings {
   Settings() {}
   uint8_t keyscan_interval         = 15;
   uint8_t led_brightness_ledDriver = 255;
   uint8_t led_brightness_ungerlow  = 255;
 };
 inline static Settings settings_{};
 inline static uint16_t settings_base_;

 static void setbrightness(const Settings &data);
};

Communications_protocol::Devices leftConnection[1]{UNKNOWN};
Communications_protocol::Devices rightConnection[1]{UNKNOWN};

auto checkBrightness = [](const Packet &)
{
    if(!::LEDControl.isEnabled())
    {
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
    ColormapEffectDefy.updateBrigthness(ColormapEffectDefy.no_led_effect, true, true);
};

void Hands::setup() {

 Communications.callbacks.bind(CONNECTED, (
                                            [](const Packet &p)
                                            {
                                              if (p.header.device == KEYSCANNER_DEFY_LEFT) leftConnection[0] = KEYSCANNER_DEFY_LEFT;
                                              if (p.header.device == KEYSCANNER_DEFY_RIGHT) rightConnection[0] = KEYSCANNER_DEFY_RIGHT;
                                            }));
 Communications.callbacks.bind(DISCONNECTED, (
                                               [](const Packet &p){
                                                 if (p.header.device == KEYSCANNER_DEFY_LEFT) leftConnection[0] = UNKNOWN;
                                                 if (p.header.device == KEYSCANNER_DEFY_RIGHT) rightConnection[0] = UNKNOWN;
                                               }));

 Communications.callbacks.bind(CONNECTED, ([](const Packet &) { ::LEDControl.set_mode(::LEDControl.get_mode_index()); }));


 settings_base_ = ::EEPROMSettings.requestSlice(sizeof(Settings));
 bool edited    = false;
 Settings settings;
 Runtime.storage().get(settings_base_, settings);
 if (settings.keyscan_interval == 0xff) {
   settings.keyscan_interval = settings_.keyscan_interval;
   edited                    = true;
 }

 if (settings.led_brightness_ledDriver == 0xff) {
   settings.led_brightness_ledDriver = settings_.led_brightness_ledDriver;
 }
 if (settings.led_brightness_ungerlow == 0xff) {
   settings.led_brightness_ungerlow = settings_.led_brightness_ungerlow;
 }
 if (edited) {
   Runtime.storage().put(settings_base_, settings);
 }
 Runtime.storage().get(settings_base_, settings_);
}

void Hands::setbrightness(const Settings &data)
{
    Runtime.storage().put(settings_base_, data);
    Runtime.storage().commit();
}

void Hands::setKeyscanInterval(uint8_t interval) {
 if (interval < 15) return;
 settings_.keyscan_interval = interval;
 Packet p{};
 p.header.command = Communications_protocol::KEYSCAN_INTERVAL;
 p.header.size    = 1;
 p.data[0]        = interval;
 Communications.sendPacket(p);
 Runtime.storage().put(settings_base_, settings_);
 Runtime.storage().commit();
}

void Hands::setLedBrightnessLedDriver(uint8_t brightness) {
 settings_.led_brightness_ledDriver = brightness;
 sendPacketBrightness();
 setbrightness(settings_);
}

void Hands::setLedBrightnessUG(uint8_t brightnessUG) {
 settings_.led_brightness_ungerlow = brightnessUG;
 sendPacketBrightness();
 setbrightness(settings_);
}

void Hands::sendPacketBrightness()
{
 Packet p{};
 checkBrightness(p);
}

/********* LED Driver *********/


void LedDriverWN::setBrightness(uint8_t brightness) {
 Hands::setLedBrightnessLedDriver(brightness);
}
void LedDriverWN::setBrightnessUG(uint8_t brightnessUG) {
 Hands::setLedBrightnessUG(brightnessUG);
}

uint8_t LedDriverWN::getBrightness() {
 return Hands::getLedBrightnessLedDriver();
}

uint8_t LedDriverWN::getBrightnessUG() {
 return Hands::getLedBrightnessUG();
}

void LedDriverWN::syncLeds() {
 bool is_enabled = ::LEDControl.isEnabled();

 if (leds_enabled_ != is_enabled)
 {
   leds_enabled_ = is_enabled;
   Packet p;
   checkBrightness(p);
 }

 if (isLEDChangedNeuron) {
   updateNeuronLED();
   isLEDChangedNeuron = false;
 }
}

void LedDriverWN::updateNeuronLED() {
 static constexpr struct {
   uint8_t r, g, b, w;
 } pins = {RGBW_LED_RED, RGBW_LED_GREEN, RGBW_LED_BLUE, RGBW_LED_WHITE};

 // invert as these are common anode, and make sure we reach 65535 to be able
 // to turn fully off.
 analogWrite(pins.r, (int)(neuronLED.r * (Hands::getLedBrightnessLedDriver() / (float)255)) << 8);
 analogWrite(pins.g, (int)(neuronLED.g * (Hands::getLedBrightnessLedDriver() / (float)255)) << 8);
 analogWrite(pins.b, (int)(neuronLED.b * (Hands::getLedBrightnessLedDriver() / (float)255)) << 8);
 analogWrite(pins.w, (int)(neuronLED.w * (Hands::getLedBrightnessLedDriver() / (float)255)) << 8);
}

void LedDriverWN::setCrgbAt(uint8_t i, cRGB crgb) {
 // prevent reading off the end of the led_map array
 if (i >= LedDriverProps::led_count)
   return;

 // neuron LED
 if (i == LedDriverProps::led_count - 2) {
   isLEDChangedNeuron |= !(neuronLED.r == crgb.r && neuronLED.g == crgb.g && neuronLED.b == crgb.b && neuronLED.w == crgb.w);
   neuronLED = crgb;
   return;
 }

 // get the SLED index
 uint8_t sled_num = led_map[i];
 if (sled_num < LEDS_PER_HAND) {
   cRGB oldColor                           = Hands::leftHand.led_data.leds[sled_num];
   Hands::leftHand.led_data.leds[sled_num] = crgb;
   isLEDChangedLeft[uint8_t(sled_num / 8)] |=
     !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b && oldColor.w == crgb.w);
 } else if (sled_num < 2 * LEDS_PER_HAND) {
   cRGB oldColor =
     Hands::rightHand.led_data.leds[sled_num - LEDS_PER_HAND];
   Hands::rightHand.led_data.leds[sled_num - LEDS_PER_HAND] = crgb;
   isLEDChangedRight[uint8_t((sled_num - LEDS_PER_HAND) / 8)] |=
     !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b && oldColor.w == crgb.w);
 } else {
   // TODO(anyone):
   // how do we want to handle debugging assertions about crazy user
   // code that would overwrite other memory?
 }
}

void LedDriverWN::setCrgbNeuron(cRGB crgb) {
 isLEDChangedNeuron |= !(neuronLED.r == crgb.r && neuronLED.g == crgb.g && neuronLED.b == crgb.b && neuronLED.w == crgb.w);
 neuronLED = crgb;
}

cRGB LedDriverWN::getCrgbAt(uint8_t i) {
 if (i >= LedDriverProps::led_count)
   return {0, 0, 0};

 uint8_t sled_num = led_map[i];
 if (sled_num < LEDS_PER_HAND) {
   return Hands::leftHand.led_data.leds[sled_num];
 } else if (sled_num < 2 * LEDS_PER_HAND) {
   return Hands::rightHand.led_data.leds[sled_num - LEDS_PER_HAND];
 } else {
   return {0, 0, 0, 0};
 }
}

void LedDriverWN::setup() {
 analogWriteResolution(16);
}

/********* Key scanner *********/

void KeyScannerWN::readMatrix() {
 previousLeftHandState  = leftHandState;
 previousRightHandState = rightHandState;

 if (Hands::leftHand.newKey()) {
   leftHandState = Hands::leftHand.getKeyData();
 }
 if (Hands::rightHand.newKey()) {
   rightHandState = Hands::rightHand.getKeyData();
 }
}

void KeyScannerWN::actOnMatrixScan() {
 for (byte row = 0; row < Props_::matrix_rows; row++) {
   for (byte col = 0; col < Props_::left_columns; col++) {
     uint8_t keynum = (row * Props_::left_columns) + col;
     uint8_t keyState;

     // left
     keyState = (bitRead(previousLeftHandState.all, keynum) << 0) |
                (bitRead(leftHandState.all, keynum) << 1);
     if (keyState)
       ThisType::handleKeyswitchEvent(Key_NoKey, KeyAddr(row, col), keyState);

     // right
     keyState = (bitRead(previousRightHandState.all, keynum) << 0) |
                (bitRead(rightHandState.all, keynum) << 1);
     if (keyState)
       ThisType::handleKeyswitchEvent(
         Key_NoKey, KeyAddr(row, (Props_::matrix_columns - 1) - col), keyState);
   }
 }
}

void KeyScannerWN::scanMatrix() {
 readMatrix();
 actOnMatrixScan();
}

void KeyScannerWN::maskKey(KeyAddr key_addr) {
 if (!key_addr.isValid())
   return;

 auto row = key_addr.row();
 auto col = key_addr.col();

 if (col >= Props_::left_columns) {
   rightHandMask.rows[row] |=
     1 << (Props_::right_columns - (col - Props_::left_columns));
 } else {
   leftHandMask.rows[row] |= 1 << (Props_::right_columns - col);
 }
}

void KeyScannerWN::unMaskKey(KeyAddr key_addr) {
 if (!key_addr.isValid())
   return;

 auto row = key_addr.row();
 auto col = key_addr.col();

 if (col >= Props_::left_columns) {
   rightHandMask.rows[row] &=
     ~(1 << (Props_::right_columns - (col - Props_::left_columns)));
 } else {
   leftHandMask.rows[row] &= ~(1 << (Props_::right_columns - col));
 }
}

bool KeyScannerWN::isKeyMasked(KeyAddr key_addr) {
 if (!key_addr.isValid())
   return false;

 auto row = key_addr.row();
 auto col = key_addr.col();

 if (col >= 8) {
   return rightHandMask.rows[row] & (1 << (7 - (col - 8)));
 } else {
   return leftHandMask.rows[row] & (1 << (7 - col));
 }
}

void KeyScannerWN::maskHeldKeys() {
 memcpy(leftHandMask.rows, leftHandState.rows, sizeof(leftHandMask));
 memcpy(rightHandMask.rows, rightHandState.rows, sizeof(rightHandMask));
}

bool KeyScannerWN::isKeyswitchPressed(KeyAddr key_addr) {
 auto row = key_addr.row();
 auto col = key_addr.col();

 if (col >= Props_::left_columns) {
   return (bitRead(rightHandState.rows[row],
                   (Props_::matrix_columns - 1) - col) != 0);
 } else {
   return (bitRead(leftHandState.rows[row], col) != 0);
 }
}

bool KeyScannerWN::wasKeyswitchPressed(KeyAddr key_addr) {
 auto row = key_addr.row();
 auto col = key_addr.col();

 if (col >= Props_::left_columns) {
   return (bitRead(previousRightHandState.rows[row],
                   (Props_::matrix_columns - 1) - col) != 0);
 } else {
   return (bitRead(previousLeftHandState.rows[row], col) != 0);
 }
}

uint8_t KeyScannerWN::pressedKeyswitchCount() {
 return __builtin_popcountll(leftHandState.all) +
        __builtin_popcountll(rightHandState.all);
}

uint8_t KeyScannerWN::previousPressedKeyswitchCount() {
 return __builtin_popcountll(previousLeftHandState.all) +
        __builtin_popcountll(previousRightHandState.all);
}

void KeyScannerWN::setKeyscanInterval(uint8_t interval) {
 //  Hands::leftHand.setKeyscanInterval(interval);
 //  Hands::rightHand.setKeyscanInterval(interval);
}

void KeyScannerWN::setup() {
 static constexpr uint8_t keyscanner_pins[] = {
   2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42};
 for (int i = 0; i < sizeof(keyscanner_pins); i++) {
   // pinMode(keyscanner_pins[i], OUTPUT);
   // digitalWrite(keyscanner_pins[i], LOW);
 }
}

void KeyScannerWN::reset() {
 leftHandState.all  = 0;
 rightHandState.all = 0;
 Runtime.hid().keyboard().releaseAllKeys();
 Runtime.hid().keyboard().sendReport();
}

/********* Hardware plugin *********/
void DefyWN::setup() {
 gpio_init(SIDE_nRESET_1);
 gpio_init(SIDE_nRESET_2);
 DefyWN::side::reset_sides();
 Hands::setup();
 KeyScannerWN::setup();
 LedDriverWN::setup();
}

void DefyWN::side::prepareForFlash() {
 WIRE_.end();
 // also turn off i2c pins to stop attiny from getting enough current through
 // i2c to stay on
 pinMode(I2C_SDA_PIN, OUTPUT);
 pinMode(I2C_SCL_PIN, OUTPUT);
 digitalWrite(I2C_SDA_PIN, false);
 digitalWrite(I2C_SCL_PIN, false);
 // wipe key states, to prevent accidental key repeats
 KeyScanner::reset();

 WIRE_.setSDA(I2C_SDA_PIN);
 WIRE_.setSCL(I2C_SCL_PIN);
 WIRE_.begin();
 WIRE_.setClock(I2C_FLASH_CLOCK_KHZ * 1000);
}

uint8_t DefyWN::side::getPower() {
 return digitalRead(SIDE_nRESET_1) && digitalRead(SIDE_nRESET_2);
}
void DefyWN::side::setPower(bool power) {
 if (power) {
   gpio_set_dir(SIDE_nRESET_2, GPIO_IN);
   gpio_pull_up(SIDE_nRESET_2);
   gpio_set_dir(SIDE_nRESET_1, GPIO_IN);
   gpio_pull_up(SIDE_nRESET_1);
 } else {
   gpio_set_dir(SIDE_nRESET_2, GPIO_OUT);
   gpio_put(SIDE_nRESET_2, false);
   gpio_set_dir(SIDE_nRESET_1, GPIO_OUT);
   gpio_put(SIDE_nRESET_1, false);
 }
}


void DefyWN::side::reset_sides() {
 gpio_set_dir(SIDE_nRESET_1, GPIO_OUT);
 gpio_set_dir(SIDE_nRESET_2, GPIO_OUT);
 gpio_put(SIDE_nRESET_1, false);
 gpio_put(SIDE_nRESET_2, false);
 sleep_ms(10);
 gpio_set_dir(SIDE_nRESET_2, GPIO_IN);
 gpio_set_dir(SIDE_nRESET_1, GPIO_IN);
 gpio_pull_up(SIDE_nRESET_1);
 gpio_pull_up(SIDE_nRESET_2);
 sleep_ms(50);  //Back to 10 ms just in case we dont miss packets //Put this to 50ms once we change the sending of the messages to be the nueron no the keyscanner
}

std::string DefyWN::getChipID() {
 char buf[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
 pico_get_unique_board_id_string(buf, sizeof(buf));
 return {buf};
}

uint8_t DefyWN::settings::keyscanInterval() {
 return Hands::getKeyscanInterval();
}
void DefyWN::settings::keyscanInterval(uint8_t interval) {
 Hands::setKeyscanInterval(interval);
}
int LedDriverWN::getBrightnessWireless() {
 return getBrightness();
}
uint8_t LedDriverWN::getBrightnessUGWireless() {
 return getBrightnessUG();
}
Devices KeyScannerWN::leftHandDevice() {
 return leftConnection[0];
}
Devices KeyScannerWN::rightHandDevice() {
 return rightConnection[0];
}

bool KeyScannerWN::rightSideWiredConnection() {
return gpio_get(SIDE_nRESET_2);
}

bool KeyScannerWN::leftSideWiredConnection(){
  return gpio_get(SIDE_nRESET_1);
}

}  // namespace dygma
}  // namespace device
}  // namespace kaleidoscope

#endif
