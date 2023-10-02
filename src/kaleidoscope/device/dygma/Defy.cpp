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

#ifdef ARDUINO_NRF5_DEFY

#include "kaleidoscope/Runtime.h"
#include <Kaleidoscope-EEPROM-Settings.h>
#include <Kaleidoscope-LEDControl.h>
#include <KeyboardioHID.h>
#include <Wire.h>

#include "kaleidoscope/util/crc16.h"
#include "kaleidoscope/driver/color/GammaCorrection.h"
#include "kaleidoscope/driver/keyscanner/Base_Impl.h"

#define I2C_CLOCK_KHZ 200
#define I2C_FLASH_CLOCK_KHZ 100  // flashing doesn't work reliably at higher clock speeds

#define SIDE_POWER 1  // side power switch pa10

#define LAYOUT_ISO 0
#define LAYOUT_ANSI 1

namespace kaleidoscope {
namespace device {
namespace dygma {

/********* DefyHands *********/

struct DefyHands {
  static Defy::Hand leftHand;
  static Defy::Hand rightHand;

  static void setup();
  static void initializeSides();

  static uint8_t layout;

  static void setSidePower(bool power);
  static bool getSidePower() { return side_power_; }

  static void keyscanInterval(uint16_t interval);
  static uint16_t keyscanInterval() { return keyscan_interval_; }

  static String getChipID();

  static void ledBrightnessCorrection(uint8_t brightness);
  static uint8_t ledBrightnessCorrection() {
    return led_brightness_correction_;
  }

 private:
  static uint16_t keyscan_interval_;
  static uint8_t led_brightness_correction_;
  static bool side_power_;
  static uint16_t settings_base_;
  static uint16_t settings_brightness_;
  static constexpr uint8_t iso_only_led_ = 19;
};

Defy::Hand DefyHands::leftHand(0);
Defy::Hand DefyHands::rightHand(1);
uint8_t DefyHands::layout;
bool DefyHands::side_power_;
uint16_t DefyHands::settings_base_;
uint16_t DefyHands::settings_brightness_;
uint8_t DefyHands::led_brightness_correction_ = 255;
uint16_t DefyHands::keyscan_interval_ = 50;

void DefyHands::setSidePower(bool power) {
  digitalWrite(SIDE_POWER, power ? HIGH : LOW);
  side_power_ = power;
}

void DefyHands::setup() {
  settings_base_ = ::EEPROMSettings.requestSlice(sizeof(keyscan_interval_));
  settings_brightness_ =
      ::EEPROMSettings.requestSlice(sizeof(led_brightness_correction_));

  // If keyscan is max, assume that EEPROM is uninitialized, and store the
  // defaults.
  uint16_t interval;
  Runtime.storage().get(settings_base_, interval);
  if (interval == 0xffff) {
    Runtime.storage().put(settings_base_, keyscan_interval_);
    Runtime.storage().commit();
  }
  Runtime.storage().get(settings_base_, keyscan_interval_);

  uint8_t brightness;
  Runtime.storage().get(settings_brightness_, brightness);
  if (brightness == 0xff) {
    Runtime.storage().put(settings_brightness_, led_brightness_correction_);
    Runtime.storage().commit();
  }
  Runtime.storage().get(settings_brightness_, led_brightness_correction_);
}

void DefyHands::keyscanInterval(uint16_t interval) {
  leftHand.setKeyscanInterval(interval);
  rightHand.setKeyscanInterval(interval);
  keyscan_interval_ = interval;
  Runtime.storage().put(settings_base_, keyscan_interval_);
  Runtime.storage().commit();
}

void DefyHands::ledBrightnessCorrection(uint8_t brightness) {
  leftHand.setBrightness(brightness);
  rightHand.setBrightness(brightness);
  led_brightness_correction_ = brightness;
  Runtime.storage().put(settings_brightness_, led_brightness_correction_);
  Runtime.storage().commit();
}

String DefyHands::getChipID() {
  uint32_t pdwUniqueID[4];
  pdwUniqueID[0] = *(volatile uint32_t*)(0x0080A00C);
  pdwUniqueID[1] = *(volatile uint32_t*)(0x0080A040);
  pdwUniqueID[2] = *(volatile uint32_t*)(0x0080A044);
  pdwUniqueID[3] = *(volatile uint32_t*)(0x0080A048);
  char buf[33];
  snprintf(buf, sizeof(buf), "%8x%8x%8x%8x",
  pdwUniqueID[0], pdwUniqueID[1], pdwUniqueID[2], pdwUniqueID[3]);
  return String(buf);
}

void DefyHands::initializeSides() {
  // key scan interval from eeprom
  leftHand.setKeyscanInterval(keyscan_interval_);
  rightHand.setKeyscanInterval(keyscan_interval_);

  // led brightness from eeprom
  leftHand.setBrightness(led_brightness_correction_);
  rightHand.setBrightness(led_brightness_correction_);

  // get ANSI/ISO at every side replug
  uint8_t l_layout = leftHand.readLayout();
  uint8_t r_layout = rightHand.readLayout();

  // setup layout variable, this will affect led mapping - defaults to ISO if
  // nothing reported
  // FIXME
  if (l_layout == 1 || r_layout == 1)
    layout = 1;
  else
    layout = 0;

  /*
   * if the neuron starts up with no sides connected, it will assume ISO. This
   * turns on an extra LED (hardware LED 19 on left side). If an ANSI left is
   * then plugged in, the keyboard will switch to ANSI, but LED 19 can't get
   * wiped because the ANSI LED map doesn't include this LED. It will be driven
   * from the SLED1735's memory with the same colour as before, which causes
   * weird looking colours to come on on other seemingly unrelated keys. So: on
   * a replug, set LED 19 to off to be safe.
   */
  leftHand.led_data.leds[iso_only_led_] = {0, 0, 0};

  // get activated LED plugin to refresh
  ::LEDControl.refreshAll();
}

/********* LED Driver *********/

bool DefyLEDDriver::isLEDChangedNeuron;
uint8_t DefyLEDDriver::isLEDChangedLeft[LED_BANKS];
uint8_t DefyLEDDriver::isLEDChangedRight[LED_BANKS];
cRGB DefyLEDDriver::neuronLED;
constexpr uint8_t DefyLEDDriver::led_map[][DefyLEDDriverProps::led_count + 1];

constexpr uint8_t DefyLEDDriverProps::key_led_map[];

void DefyLEDDriver::setBrightness(uint8_t brightness) {
  DefyHands::ledBrightnessCorrection(brightness);
  for (uint8_t i = 0; i < LED_BANKS; i++) {
    isLEDChangedLeft[i] = true;
    isLEDChangedRight[i] = true;
  }
}

uint8_t DefyLEDDriver::getBrightness() {
  return DefyHands::ledBrightnessCorrection();
}

void DefyLEDDriver::syncLeds() {
  // left and right sides
  for (uint8_t i = 0; i < LED_BANKS; i++) {
    // only send the banks that have changed - try to improve jitter performance
    if (isLEDChangedLeft[i]) {
      DefyHands::leftHand.sendLEDBank(i);
      isLEDChangedLeft[i] = false;
    }
    if (isLEDChangedRight[i]) {
      DefyHands::rightHand.sendLEDBank(i);
      isLEDChangedRight[i] = false;
    }
  }

  if (isLEDChangedNeuron) {
    updateNeuronLED();
    isLEDChangedNeuron = false;
  }
}

void DefyLEDDriver::updateNeuronLED() {
  static constexpr struct {
    uint8_t r, g, b;
  } pins = { 3, 5, 4 };
  auto constexpr gamma8 = kaleidoscope::driver::color::gamma_correction;

  // invert as these are common anode, and make sure we reach 65535 to be able
  // to turn fully off.
  analogWrite(pins.r, ((256 - pgm_read_byte(&gamma8[neuronLED.r])) << 8) - 1);
  analogWrite(pins.g, ((256 - pgm_read_byte(&gamma8[neuronLED.g])) << 8) - 1);
  analogWrite(pins.b, ((256 - pgm_read_byte(&gamma8[neuronLED.b])) << 8) - 1);
}

void DefyLEDDriver::setCrgbAt(uint8_t i, cRGB crgb) {
  // prevent reading off the end of the led_map array
  if (i >= DefyLEDDriverProps::led_count) return;

  // neuron LED
  if (i == DefyLEDDriverProps::led_count - 1) {
    isLEDChangedNeuron |= !(neuronLED.r == crgb.r && neuronLED.g == crgb.g &&
                            neuronLED.b == crgb.b);
    neuronLED = crgb;
    return;
  }

  // get the SLED index
  uint8_t sled_num = led_map[DefyHands::layout][i];
  if (sled_num < LEDS_PER_HAND) {
    cRGB oldColor = DefyHands::leftHand.led_data.leds[sled_num];
    DefyHands::leftHand.led_data.leds[sled_num] = crgb;
    isLEDChangedLeft[uint8_t(sled_num / 8)] |=
        !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b);
  } else if (sled_num < 2 * LEDS_PER_HAND) {
    cRGB oldColor =
        DefyHands::rightHand.led_data.leds[sled_num - LEDS_PER_HAND];
    DefyHands::rightHand.led_data.leds[sled_num - LEDS_PER_HAND] = crgb;
    isLEDChangedRight[uint8_t((sled_num - LEDS_PER_HAND) / 8)] |=
        !(oldColor.r == crgb.r && oldColor.g == crgb.g && oldColor.b == crgb.b);
  } else {
    // TODO(anyone):
    // how do we want to handle debugging assertions about crazy user
    // code that would overwrite other memory?
  }
}

cRGB DefyLEDDriver::getCrgbAt(uint8_t i) {
  if (i >= DefyLEDDriverProps::led_count) return {0, 0, 0};

  uint8_t sled_num = led_map[DefyHands::layout][i];
  if (sled_num < LEDS_PER_HAND) {
    return DefyHands::leftHand.led_data.leds[sled_num];
  } else if (sled_num < 2 * LEDS_PER_HAND) {
    return DefyHands::rightHand.led_data.leds[sled_num - LEDS_PER_HAND];
  } else {
    return {0, 0, 0};
  }
}

void DefyLEDDriver::setup() {
  pinMode(SIDE_POWER, OUTPUT);
  DefyHands::setSidePower(false);

  // arduino zero analogWrite(255) isn't fully on as its actually working with a
  // 16bit counter and the mapping is a bit shift.
  // so change to 16 bit resolution to avoid the mapping and do the mapping
  // ourselves in updateHubleLED() to ensure LEDs can be set fully off
  analogWriteResolution(16);
  updateNeuronLED();

  delay(10);
  DefyHands::setSidePower(true);
  delay(500);  // wait for sides to power up and finish bootloader
}

/********* Key scanner *********/

Defy::keydata_t DefyKeyScanner::leftHandState;
Defy::keydata_t DefyKeyScanner::rightHandState;
Defy::keydata_t DefyKeyScanner::previousLeftHandState;
Defy::keydata_t DefyKeyScanner::previousRightHandState;
Defy::keydata_t DefyKeyScanner::leftHandMask;
Defy::keydata_t DefyKeyScanner::rightHandMask;
bool DefyKeyScanner::lastLeftOnline;
bool DefyKeyScanner::lastRightOnline;

void DefyKeyScanner::readMatrix() {
  previousLeftHandState = leftHandState;
  previousRightHandState = rightHandState;

  if (DefyHands::leftHand.readKeys()) {
    leftHandState = DefyHands::leftHand.getKeyData();
    // if ANSI, then swap r3c0 and r3c1 to match the PCB
    if (DefyHands::layout == LAYOUT_ANSI) {
      // only swap if bits are different
      if ((leftHandState.rows[3] & (1 << 0)) ^
          leftHandState.rows[3] & (1 << 1)) {
        leftHandState.rows[3] ^= (1 << 0);  // flip the bit
        leftHandState.rows[3] ^= (1 << 1);  // flip the bit
      }
    }
  }

  if (DefyHands::rightHand.readKeys()) {
    rightHandState = DefyHands::rightHand.getKeyData();
    // if ANSI, then swap r1c0 and r2c0 to match the PCB
    if (DefyHands::layout == LAYOUT_ANSI) {
      if ((rightHandState.rows[1] & (1 << 0)) ^
          rightHandState.rows[2] & (1 << 0)) {
        rightHandState.rows[1] ^= (1 << 0);
        rightHandState.rows[2] ^= (1 << 0);
      }
    }
  }

  // if a side has just been replugged, initialise it
  if ((DefyHands::leftHand.online && !lastLeftOnline) ||
      (DefyHands::rightHand.online && !lastRightOnline))
    DefyHands::initializeSides();

  // if a side has just been unplugged, wipe its state
  if (!DefyHands::leftHand.online && lastLeftOnline) leftHandState.all = 0;

  if (!DefyHands::rightHand.online && lastRightOnline) rightHandState.all = 0;

  // store previous state of whether the sides are plugged in
  lastLeftOnline = DefyHands::leftHand.online;
  lastRightOnline = DefyHands::rightHand.online;
}

void DefyKeyScanner::actOnMatrixScan() {
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
            Key_NoKey, KeyAddr(row, (Props_::matrix_columns - 1) - col),
            keyState);
    }
  }
}

void DefyKeyScanner::scanMatrix() {
  readMatrix();
  actOnMatrixScan();
}

void DefyKeyScanner::maskKey(KeyAddr key_addr) {
  if (!key_addr.isValid()) return;

  auto row = key_addr.row();
  auto col = key_addr.col();

  if (col >= Props_::left_columns) {
    rightHandMask.rows[row] |=
        1 << (Props_::right_columns - (col - Props_::left_columns));
  } else {
    leftHandMask.rows[row] |= 1 << (Props_::right_columns - col);
  }
}

void DefyKeyScanner::unMaskKey(KeyAddr key_addr) {
  if (!key_addr.isValid()) return;

  auto row = key_addr.row();
  auto col = key_addr.col();

  if (col >= Props_::left_columns) {
    rightHandMask.rows[row] &=
        ~(1 << (Props_::right_columns - (col - Props_::left_columns)));
  } else {
    leftHandMask.rows[row] &= ~(1 << (Props_::right_columns - col));
  }
}

bool DefyKeyScanner::isKeyMasked(KeyAddr key_addr) {
  if (!key_addr.isValid()) return false;

  auto row = key_addr.row();
  auto col = key_addr.col();

  if (col >= 8) {
    return rightHandMask.rows[row] & (1 << (7 - (col - 8)));
  } else {
    return leftHandMask.rows[row] & (1 << (7 - col));
  }
}

void DefyKeyScanner::maskHeldKeys() {
  memcpy(leftHandMask.rows, leftHandState.rows, sizeof(leftHandMask));
  memcpy(rightHandMask.rows, rightHandState.rows, sizeof(rightHandMask));
}

bool DefyKeyScanner::isKeyswitchPressed(KeyAddr key_addr) {
  auto row = key_addr.row();
  auto col = key_addr.col();

  if (col >= Props_::left_columns) {
    return (bitRead(rightHandState.rows[row],
                    (Props_::matrix_columns - 1) - col) != 0);
  } else {
    return (bitRead(leftHandState.rows[row], col) != 0);
  }
}

bool DefyKeyScanner::wasKeyswitchPressed(KeyAddr key_addr) {
  auto row = key_addr.row();
  auto col = key_addr.col();

  if (col >= Props_::left_columns) {
    return (bitRead(previousRightHandState.rows[row],
                    (Props_::matrix_columns - 1) - col) != 0);
  } else {
    return (bitRead(previousLeftHandState.rows[row], col) != 0);
  }
}

uint8_t DefyKeyScanner::pressedKeyswitchCount() {
  return __builtin_popcountll(leftHandState.all) +
         __builtin_popcountll(rightHandState.all);
}

uint8_t DefyKeyScanner::previousPressedKeyswitchCount() {
  return __builtin_popcountll(previousLeftHandState.all) +
         __builtin_popcountll(previousRightHandState.all);
}

void DefyKeyScanner::setKeyscanInterval(uint8_t interval) {
  DefyHands::leftHand.setKeyscanInterval(interval);
  DefyHands::rightHand.setKeyscanInterval(interval);
}

void DefyKeyScanner::setup() {
  static constexpr uint8_t keyscanner_pins[] = {
      2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14,
      15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 30,
      31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42};
  for (int i = 0; i < sizeof(keyscanner_pins); i++) {
    pinMode(keyscanner_pins[i], OUTPUT);
    digitalWrite(keyscanner_pins[i], LOW);
  }
}

void DefyKeyScanner::reset() {
  leftHandState.all = 0;
  rightHandState.all = 0;
  Runtime.hid().keyboard().releaseAllKeys();
  Runtime.hid().keyboard().sendReport();
}

/********* Hardware plugin *********/
void Defy::setup() {
  DefyHands::setup();
  KeyScanner::setup();
  LEDDriver::setup();

  // initialise Wire of scanner - have to do this here to avoid problem with
  // static object intialisation ordering
  Wire.begin();
  Wire.setClock(I2C_CLOCK_KHZ * 1000);

  DefyHands::initializeSides();
}

void Defy::side::prepareForFlash() {
  Wire.end();

  setPower(LOW);
  // also turn off i2c pins to stop attiny from getting enough current through
  // i2c to stay on
  pinMode(SCL, OUTPUT);
  pinMode(SDA, OUTPUT);
  digitalWrite(SCL, false);
  digitalWrite(SDA, false);

  // wipe key states, to prevent accidental key repeats
  DefyKeyScanner::reset();

  setPower(HIGH);

  Wire.begin();
  Wire.setClock(I2C_FLASH_CLOCK_KHZ * 1000);
  // wait for side bootloader to be ready
  delay(100);
}

uint8_t Defy::side::getPower() { return DefyHands::getSidePower(); }
void Defy::side::setPower(uint8_t power) { DefyHands::setSidePower(power); }

uint8_t Defy::side::leftVersion() {
  return DefyHands::leftHand.readVersion();
}
uint8_t Defy::side::rightVersion() {
  return DefyHands::rightHand.readVersion();
}

uint8_t Defy::side::leftCRCErrors() {
  return DefyHands::leftHand.crc_errors();
}
uint8_t Defy::side::rightCRCErrors() {
  return DefyHands::rightHand.crc_errors();
}

uint8_t Defy::side::leftSLEDVersion() {
  return DefyHands::leftHand.readSLEDVersion();
}
uint8_t Defy::side::rightSLEDVersion() {
  return DefyHands::rightHand.readSLEDVersion();
}

uint8_t Defy::side::leftSLEDCurrent() {
  return DefyHands::leftHand.readSLEDCurrent();
}
uint8_t Defy::side::rightSLEDCurrent() {
  return DefyHands::rightHand.readSLEDCurrent();
}
void Defy::side::setSLEDCurrent(uint8_t current) {
  DefyHands::rightHand.setSLEDCurrent(current);
  DefyHands::leftHand.setSLEDCurrent(current);
}

Defy::settings::Layout Defy::settings::layout() {
  return DefyHands::layout == LAYOUT_ANSI ? Layout::ANSI : Layout::ISO;
}
uint8_t Defy::settings::joint() { return DefyHands::rightHand.readJoint(); }

uint16_t Defy::settings::keyscanInterval() {
  return DefyHands::keyscanInterval();
}
String Defy::settings::getChipID() {
  return DefyHands::getChipID();
}
void Defy::settings::keyscanInterval(uint16_t interval) {
  DefyHands::keyscanInterval(interval);
}

}  // namespace dygma
}  // namespace device
}  // namespace kaleidoscope

#endif
