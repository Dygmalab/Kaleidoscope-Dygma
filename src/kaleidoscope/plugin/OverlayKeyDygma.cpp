/* -*- mode: c++ -*-
 * Kaleidoscope-OverlayKeyDygma -- OVERLAY key plugin for Dygma keyboards
 * Copyright (C) 2026 Dygma Lab S.L. www.dygma.com
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

#include "OverlayKeyDygma.h"
#include "kaleidoscope/keyswitch_state.h"
#include "kaleidoscope/layers.h"
#include "KeyboardioHID.h"
#include "Adafruit_TinyUSB.h"   // TinyUSBDevice.mounted()

// The wired neuron has no radio, so it always reports over USB.
#ifndef NEURON_WIRED
#include "ble_hid_service.h"    // INPUT_REPORT_LEN_RAW
#include "Ble_composite_dev.h"  // ble_connected()
#endif

#define DEBUG_LOG_OVERLAY_KEY 0

#if DEBUG_LOG_OVERLAY_KEY && !defined(NEURON_WIRED)
#ifdef __cplusplus
extern "C" {
#endif
#include "nrf_log.h"
#ifdef __cplusplus
}
#endif
#define OVERLAY_LOG(...) NRF_LOG_INFO(__VA_ARGS__)
#else
#define OVERLAY_LOG(...) do {} while (0)
#endif

// USB Full Speed endpoint max is 64 bytes; 1 byte is the report ID, so 63 data bytes.
// Must match USB_RAW_HID_REPORT_SIZE in the HID descriptor.
#ifndef USB_RAW_HID_REPORT_SIZE
#define USB_RAW_HID_REPORT_SIZE  63
#endif

// Packet framing — must match constants.ts in Dygma-Lens
#define OVERLAY_MAGIC_BYTE       0xAA
#define PACKET_TYPE_OVERLAY      0x01  // OVERLAY_KEY (superkey with tap/hold/double-tap)
#define PACKET_TYPE_LAYER        0x02
#define PACKET_TYPE_OVERLAY_TAP  0x03  // OVERLAY_TAP key (simple tap key)
#define PACKET_TYPE_OVERLAY_HOLD 0x04  // OVERLAY_HOLD key (simple hold key)

#define OVERLAY_EVENT_RELEASE    0x00
#define OVERLAY_EVENT_TAP        0x01
#define OVERLAY_EVENT_HOLD       0x02
#define OVERLAY_EVENT_DOUBLE_TAP 0x03

namespace kaleidoscope {
namespace plugin {

// ---- HID send helpers -------------------------------------------------------

void OverlayKeyDygma::sendPacket(uint8_t packet_type, uint8_t value) {
    OVERLAY_LOG("OverlayKeyDygma: type=0x%02X value=0x%02X", packet_type, value);

#ifndef NEURON_WIRED
    if (ble_connected()) {
        uint8_t buf[INPUT_REPORT_LEN_RAW] = {};
        buf[0] = OVERLAY_MAGIC_BYTE;
        buf[1] = packet_type;
        buf[2] = value;
        HID().SendReport(HID_REPORTID_RAWHID, buf, INPUT_REPORT_LEN_RAW);
        return;
    }
#endif

    uint8_t buf[USB_RAW_HID_REPORT_SIZE] = {};
    buf[0] = OVERLAY_MAGIC_BYTE;
    buf[1] = packet_type;
    buf[2] = value;
    HID().SendReport(HID_REPORTID_RAWHID, buf, USB_RAW_HID_REPORT_SIZE);
}

// ---- Overlay key state machine -----------------------------------------------

EventHandlerResult OverlayKeyDygma::onKeyswitchEvent(Key &mapped_key, KeyAddr key_addr, uint8_t key_state) {
    // Handle OVERLAY_TAP - simple tap key (sends PACKET_TYPE_OVERLAY_TAP)
    if (mapped_key.getRaw() == kaleidoscope::ranges::OVERLAY_TAP) {
        if (keyToggledOn(key_state)) {
            sendPacket(PACKET_TYPE_OVERLAY_TAP, OVERLAY_EVENT_TAP);
        } else if (keyToggledOff(key_state)) {
            sendPacket(PACKET_TYPE_OVERLAY_TAP, OVERLAY_EVENT_RELEASE);
        }
        return EventHandlerResult::EVENT_CONSUMED;
    }

    // Handle OVERLAY_HOLD - simple hold key (sends PACKET_TYPE_OVERLAY_HOLD)
    if (mapped_key.getRaw() == kaleidoscope::ranges::OVERLAY_HOLD) {
        if (keyToggledOn(key_state)) {
            sendPacket(PACKET_TYPE_OVERLAY_HOLD, OVERLAY_EVENT_HOLD);
        } else if (keyToggledOff(key_state)) {
            sendPacket(PACKET_TYPE_OVERLAY_HOLD, OVERLAY_EVENT_RELEASE);
        }
        return EventHandlerResult::EVENT_CONSUMED;
    }

    // Handle OVERLAY_KEY - original tap/hold/double-tap state machine
    if (mapped_key.getRaw() != kaleidoscope::ranges::OVERLAY_KEY) {
        return EventHandlerResult::OK;
    }

    if (keyToggledOn(key_state)) {
        switch (overlay_state_) {
            case State::IDLE:
                overlay_state_  = State::PRESSED;
                press_start_ms_ = Runtime.millisAtCycleStart();
                break;

            case State::FIRST_UP:
                // Second press within TAP_WINDOW_MS → double tap
                overlay_state_ = State::DOUBLE_PRESSED;
                sendPacket(PACKET_TYPE_OVERLAY, OVERLAY_EVENT_DOUBLE_TAP);
                break;

            default:
                break;
        }
    } else if (keyToggledOff(key_state)) {
        switch (overlay_state_) {
            case State::PRESSED:
                // Released before hold timeout → wait to see if it's a tap or double-tap start
                overlay_state_ = State::FIRST_UP;
                release_ms_    = Runtime.millisAtCycleStart();
                break;

            case State::HOLD_ACTIVE:
            case State::DOUBLE_PRESSED:
                overlay_state_ = State::IDLE;
                sendPacket(PACKET_TYPE_OVERLAY, OVERLAY_EVENT_RELEASE);
                break;

            default:
                break;
        }
    }

    // Consume so the host never sees this key as a normal keycode
    return EventHandlerResult::EVENT_CONSUMED;
}

// ---- Periodic tick -----------------------------------------------------------

EventHandlerResult OverlayKeyDygma::beforeReportingState() {
    // Send initial layer once a HID connection is established
#ifdef NEURON_WIRED
    const bool host_connected = TinyUSBDevice.mounted();
#else
    const bool host_connected = ble_connected() || TinyUSBDevice.mounted();
#endif

    if (!initial_sent_ && host_connected) {
        initial_sent_ = true;
        sendPacket(PACKET_TYPE_LAYER, Layer.mostRecent());
    }

    // Resolve pending overlay key states by timer
    switch (overlay_state_) {
        case State::PRESSED:
            if (Runtime.hasTimeExpired(press_start_ms_, HOLD_TIMEOUT_MS)) {
                overlay_state_ = State::HOLD_ACTIVE;
                sendPacket(PACKET_TYPE_OVERLAY, OVERLAY_EVENT_HOLD);
            }
            break;

        case State::FIRST_UP:
            if (Runtime.hasTimeExpired(release_ms_, TAP_WINDOW_MS)) {
                overlay_state_ = State::IDLE;
                sendPacket(PACKET_TYPE_OVERLAY, OVERLAY_EVENT_TAP);
            }
            break;

        default:
            break;
    }

    return EventHandlerResult::OK;
}

// ---- Layer change -----------------------------------------------------------

EventHandlerResult OverlayKeyDygma::onLayerChange() {
    sendPacket(PACKET_TYPE_LAYER, Layer.mostRecent());
    return EventHandlerResult::OK;
}

}
}

kaleidoscope::plugin::OverlayKeyDygma OverlayKey;
