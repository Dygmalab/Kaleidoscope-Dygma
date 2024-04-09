
#pragma once

#ifdef NRF52_ARCH

#include "Adafruit_TinyUSB.h"
#include "Arduino.h"
#include "EEPROM.h"
#include "bootloader_util.h"
#include "kaleidoscope/driver/bootloader/Base.h"
#include "nrf_sdh.h"


namespace kaleidoscope {
namespace driver {
namespace bootloader {
namespace nrf {

class nrfBoot : public kaleidoscope::driver::bootloader::Base {
   public:
    static void rebootBootloader() {
        EEPROM.erase();
        TinyUSBDevice.detach();
        bldrUtil::init();
        bldrUtil::update_request();
        delay(100);
        nrf_sdh_disable_request();
        reset_mcu();
    }
};

}  // namespace nrf
}  // namespace bootloader
}  // namespace driver
}  // namespace kaleidoscope

#endif
