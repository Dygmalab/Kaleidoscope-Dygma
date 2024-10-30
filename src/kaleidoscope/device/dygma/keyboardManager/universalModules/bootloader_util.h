
#ifndef __BOOTLOADER_UTIL_H_
#define __BOOTLOADER_UTIL_H_

#include <stdbool.h>
#include <stdint.h>

class bldrUtil
{
    public:

        static bool init( void );
        static bool update_request( void );

    private:
};

#endif /* __BOOTLOADER_UTIL_H_ */
