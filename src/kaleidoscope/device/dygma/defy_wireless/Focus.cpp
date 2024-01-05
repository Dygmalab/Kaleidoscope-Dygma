/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::defy_wireless -- Kaleidoscope device plugin for Dygma defy_wireless
 * Copyright (C) 2017-2019  Keyboard.io, Inc
 * Copyright (C) 2017-2019  Dygma Lab S.L.
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
#include <Kaleidoscope-FocusSerial.h>
#include "kaleidoscope/device/dygma/raise2/Focus.h"
#include "Communications.h"

namespace kaleidoscope {
namespace device {
namespace dygma {
namespace defy_wireless {

#ifndef Defy_FIRMWARE_VERSION
#define Defy_FIRMWARE_VERSION "<unknown>"
#endif

EventHandlerResult Focus::onFocusEvent(const char *command) {
    if (::Focus.handleHelp(command,
                           "hardware.version\n"
                           "hardware.side_power\n"
                           "hardware.side_ver\n"
                           "hardware.keyscanInterval\n"
                           "hardware.firmware\n"
                           "hardware.chip_id\n"
                           "hardware.chip_info"))
        return EventHandlerResult::OK;

    if (strncmp(command, "hardware.", 9) != 0) {
        return EventHandlerResult::OK;
    }

    if (strcmp(command + 9, "sideLeft") == 0) {
        auto &keyScanner = Runtime.device().keyScanner();
        auto deviceLeft = keyScanner.leftHandDevice();
        ::Focus.send(static_cast<uint8_t>(deviceLeft));
        return EventHandlerResult::OK;
    }

    if (strcmp(command + 9, "sideRight") == 0) {
        auto &keyScanner = Runtime.device().keyScanner();
        auto deviceRight = keyScanner.rightHandDevice();
        ::Focus.send(static_cast<uint8_t>(deviceRight));
        return EventHandlerResult::OK;
    }

    if (strcmp(command + 9, "version") == 0) {
        NRF_LOG_DEBUG("read request: hardware.version");

        ::Focus.send<char *>("Dygma Defy Wireless");

        return EventHandlerResult::EVENT_CONSUMED;
    }

    if (strcmp(command + 9, "firmware") == 0) {
        NRF_LOG_DEBUG("read request: hardware.firmware");

        ::Focus.send<char *>(RAISE_2_FIRMWARE_VERSION);

        return EventHandlerResult::EVENT_CONSUMED;
    }

    if (strcmp(command + 9, "chip_id") == 0) {
        NRF_LOG_DEBUG("read request: hardware.chip_id");

        char chip_id[17];
        Runtime.device().settings.getChipID(chip_id, sizeof(chip_id));
        ::Focus.send<char *>(chip_id);

        return EventHandlerResult::EVENT_CONSUMED;
    }

    if (strcmp(command + 9, "chip_info") == 0) {
        NRF_LOG_DEBUG("read request: hardware.chip_info");

        char chip_info[100];
        Runtime.device().settings.get_chip_info(chip_info, sizeof(chip_info));
        ::Focus.send<char *>(chip_info);

        return EventHandlerResult::EVENT_CONSUMED;
    }

    if (strcmp(command + 9, "side_power") == 0) {
        if (::Focus.isEOL()) {
            NRF_LOG_DEBUG("read request: hardware.side_power");

            ::Focus.send(Runtime.device().side.getPower());

            return EventHandlerResult::EVENT_CONSUMED;
        } else {
            NRF_LOG_DEBUG("write request: hardware.side_power");

            uint8_t power;
            ::Focus.read(power);
            Runtime.device().side.setPower(power);

            return EventHandlerResult::EVENT_CONSUMED;
        }
    }

    if (strcmp(command + 9, "side_ver") == 0) {
        NRF_LOG_DEBUG("read request: hardware.side_ver");
        ::Focus.send(Runtime.device().side.leftVersion());
        ::Focus.send(Runtime.device().side.rightVersion());
        return EventHandlerResult::EVENT_CONSUMED;
    }

    if (strcmp(command + 9, "keyscanInterval") == 0) {
        if (::Focus.isEOL()) {
            NRF_LOG_DEBUG("read request: hardware.keyscanInterval");

            ::Focus.send(Runtime.device().settings.keyscanInterval());

            return EventHandlerResult::EVENT_CONSUMED;
        } else {
            NRF_LOG_DEBUG("write request: hardware.keyscanInterval");

            uint8_t keyscan;
            ::Focus.read(keyscan);
            Runtime.device().settings.keyscanInterval(keyscan);

            return EventHandlerResult::EVENT_CONSUMED;
        }
    }

    return EventHandlerResult::OK;
}

void Focus::init() {
}

}  // namespace defy_wireless
}  // namespace dygma
}  // namespace device
}  // namespace kaleidoscope

kaleidoscope::device::dygma::defy_wireless::Focus DefyFocus;
#endif
