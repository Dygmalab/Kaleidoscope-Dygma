
#pragma once

#ifdef NRF52_ARCH

#include "kaleidoscope/driver/bootloader/Base.h"
#include "bootloader_drv.h"
#include "Arduino.h"


namespace kaleidoscope {
namespace driver {
namespace bootloader {
namespace nrf {

class nrfBoot : public kaleidoscope::driver::bootloader::Base {
   public:
    static void rebootBootloader() {

        bldrdrv_init();
        bldrdrv_update_request();
        reset_mcu();
    }
};

}  // namespace nrf
}  // namespace bootloader
}  // namespace driver
}  // namespace kaleidoscope

#endif
