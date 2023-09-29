#include "TWI.h"

#include "kaleidoscope/util/crc16.h"

#include "Twi_master.h"

#ifdef __cplusplus
extern "C" 
{
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#ifdef __cplusplus
}
#endif



Twi_master twi_master(TWI_SCL_PIN, TWI_SDA_PIN);


namespace kaleidoscope {
namespace device {
namespace dygma {
namespace defy_wireless {


bool TWI::twi_already_init = false;


void TWI::init(uint16_t clock_khz_)
{
    if (!TWI::twi_already_init)
    {
        // TWI speed. The nRF52833 only supports 100KHz, 250KHz and 400KHz.
        if (clock_khz_ == 100)
        {
            twi_master.init(NRF_TWIM_FREQ_100K, true);
        }
        else if (clock_khz_ == 250)
        {
            twi_master.init(NRF_TWIM_FREQ_250K, true);
        }
        else if (clock_khz_ == 400)
        {
            twi_master.init(NRF_TWIM_FREQ_400K, true);
        }
        else
        {
            NRF_LOG_ERROR("TWI can only be 100KHz, 250KHz or 400KHz.")
            NRF_LOG_FLUSH();

            while (true);
        }
    }

    TWI::twi_already_init = true;
}

uint8_t TWI::readFrom(uint8_t *buffer, uint16_t len)
{
    /*
        Blocking function.

        Parameters
        ----------
        *buffer: Pointer to a Bytes buffer where to save the received data.
        len: Number of Bytes to request.
        Returns the number of Bytes returned from the peripheral device or cero if error.

        Frame received from the keyboard (8 Bytes total length):
        [TWI_REPLY_KEYDATA, DATA_BYTE_1, .., DATA_BYTE_5, CRC16_HIGH_BYTE, CRC16_LOW_BYTE]
    */

    memset(buffer, 0, len);  // Clear buffer.
    
    #if TWI_DEBUG
    NRF_LOG_INFO("Requesting %d Bytes..", len);
    NRF_LOG_FLUSH();
    #endif

    if ( !twi_master.request_bytes(slave_addr, buffer, len) )
    {
        twi_master.reset_module();
        
        #if TWI_DEBUG
        NRF_LOG_DEBUG("<<<<< ERROR requesting Bytes, TWI reseted >>>>>");
        NRF_LOG_FLUSH();
        #endif
        
        return 0;  // in case slave is not responding - return 0 (0 length of received data).
    }

    // Wait for the transmission to get completed.
    uint64_t ti = millis();
    while (!twi_master.transfer_completed())
    {
        if ((millis() - ti) > TWI_TIMEOUT_TIME)
        {
            twi_master.reset_module();
            
            #if TWI_DEBUG
            NRF_LOG_ERROR("<<<<< ERROR requesting Bytes (Time out), TWI reseted >>>>>");
            NRF_LOG_FLUSH();
            #endif
            
            return 0;  // in case slave is not responding - return 0 (0 length of received data).
        }
    }

    #if (TWI_PRINT_RECEIVED_BUFFER_ALWAYS & !TWI_PRINT_RECEIVED_BUFFER_IF_KEYSTROKE)
    NRF_LOG_INFO("--------------");
    NRF_LOG_INFO("Slave addr: 0x%02x", slave_addr);
    NRF_LOG_INFO("Requesting %d Bytes..", len);
    for (uint32_t i = 0; i < len; i++)
    {
        NRF_LOG_INFO("buffer[%d] = %d", i, buffer[i]);
    }
    NRF_LOG_FLUSH();
    #endif

    #if (!TWI_PRINT_RECEIVED_BUFFER_ALWAYS & TWI_PRINT_RECEIVED_BUFFER_IF_KEYSTROKE)
    if (buffer[0] == 0x01)
    {
        NRF_LOG_INFO("--------------");
        NRF_LOG_INFO("Slave addr: 0x%02x", slave_addr);
        for (uint32_t i = 0; i < len; i++)
        {
            NRF_LOG_INFO("buffer[%d] = %d", i, buffer[i]);
        }
        NRF_LOG_FLUSH();
    }
    #endif
    
    if (check_crc16(buffer, len))
    {
        return len;
    }
    
    return 0;
}

bool TWI::check_crc16(uint8_t *buffer, uint16_t len)
{
    uint8_t received_crc16_high_byte = buffer[6];
    uint8_t received_crc16_low_byte = buffer[7];

    // Assemble the 16 bit CRC.
    uint16_t received_crc16 = (received_crc16_high_byte << 8) | received_crc16_low_byte;

    // Calculate the crc16 of the received data.
    uint16_t crc16 = 0xffff;
    for (uint8_t i = 0; i < (len - 2); i++) 
    {
        crc16 = _crc_ccitt_update(crc16, buffer[i]);
    }
    
    #if (TWI_PRINT_CRC16_ALWAYS & !TWI_PRINT_CRC16_IF_ERROR)
    NRF_LOG_ERROR("Received CRC = %d", received_crc16);
    NRF_LOG_ERROR("Calculated CRC = %d", crc16);
    NRF_LOG_FLUSH();
    #endif

    // And compare with the received CRC.
    if (crc16 != received_crc16)
    {
        twi_master.reset_module();

        crc_errors_++;
        
        #if (!TWI_PRINT_CRC16_ALWAYS & TWI_PRINT_CRC16_IF_ERROR)
        NRF_LOG_ERROR("CRC error");
        NRF_LOG_ERROR("received_crc16_high_byte = %d", received_crc16_high_byte);
        NRF_LOG_ERROR("received_crc16_low_byte = %d", received_crc16_low_byte);
        NRF_LOG_ERROR("Received CRC = %d", received_crc16);
        NRF_LOG_ERROR("Calculated CRC high byte = %d", (uint8_t)(crc16 >> 8));
        NRF_LOG_ERROR("Calculated CRC low byte = %d", (uint8_t)crc16);
        NRF_LOG_ERROR("Calculated CRC = %d", crc16);
        NRF_LOG_ERROR("--------------");
        NRF_LOG_FLUSH();
        #endif
        
        #if (!TWI_PRINT_CRC16_ALWAYS & !TWI_PRINT_CRC16_IF_ERROR & TWI_PRINT_CRC16_IF_ERROR_AND_KEYSTROKE)
        if (buffer[0] == 0x01)
        {
            NRF_LOG_ERROR("CRC error");
            NRF_LOG_ERROR("received_crc16_high_byte = %d", received_crc16_high_byte);
            NRF_LOG_ERROR("received_crc16_low_byte = %d", received_crc16_low_byte);
            NRF_LOG_ERROR("Received CRC = %d", received_crc16);
            NRF_LOG_ERROR("Calculated CRC high byte = %d", (uint8_t)(crc16 >> 8));
            NRF_LOG_ERROR("Calculated CRC low byte = %d", (uint8_t)crc16);
            NRF_LOG_ERROR("Calculated CRC = %d", crc16);
            NRF_LOG_ERROR("--------------");
            NRF_LOG_FLUSH();
        }
        #endif
        
        return false;
    }

    return true;
}

uint8_t TWI::writeTo(uint8_t *buffer, uint16_t len)
{
    /*
        Blocking TWI transfer.
        Return 0 if succes or 1 if not.
    */

    // Calculate the crc16 of the data.
    uint16_t crc16 = 0xffff;
    for (uint8_t i = 0; i < len; i++) 
    {
        crc16 = _crc_ccitt_update(crc16, buffer[i]);
    }

    // Get high Byte and low Byte of the crc.
    uint8_t crc_bytes[2];
    crc_bytes[0] = crc16 >> 8;
    crc_bytes[1] = crc16;

    uint8_t data[20];
    memcpy(data, buffer, len);
    memcpy(data + len, crc_bytes, 2);

    if ( !twi_master.write_bytes(slave_addr, data, len) )
    {
        twi_master.reset_module();
        
        #if TWI_DEBUG
        NRF_LOG_INFO("<<<<< ERROR writing Bytes, TWI reseted >>>>>");
        NRF_LOG_FLUSH();
        #endif
        
        return 1;
    } 
    
    // Wait for the transmission to get completed.
    uint64_t ti = millis();
    while (!twi_master.transfer_completed())
    {
        if ((millis() - ti) > TWI_TIMEOUT_TIME)
        {
            twi_master.reset_module();

            #if TWI_DEBUG
            NRF_LOG_INFO("<<<<< ERROR writing Bytes (Time out), TWI reseted >>>>>");
            NRF_LOG_FLUSH();
            #endif

            return 1;
        }
    }
    
    return 0;
}

uint8_t TWI::crc_errors(void) 
{
    return crc_errors_;
}

void TWI::recovery(void)
{
    twi_master.reset_module();
}


}
}
}
}

