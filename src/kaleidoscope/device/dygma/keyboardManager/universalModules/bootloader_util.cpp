/* -*- mode: c++ -*-
 * kaleidoscope::device::dygma::Defy -- Kaleidoscope device plugin for Dygma Defy
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

#include <string.h>
#include "bootloader_util.h"
#include "nrf_dfu_types.h"
#include "nrf_nvmc.h"
#include "utils/dl_crc32.h"

#define EXIT_IF_FALSE( err, msg ) if ( ( err ) == false ) goto _EXIT;

#define BLDR_SETTINGS_VERSION           1

#define WORD_CNT_GET( size )            ( size / sizeof(uint32_t) )

#define CRC_INITIAL_VALUE               0xFFFFFFFF

/***************************************/
/*           Settings page             */
/***************************************/
typedef struct
{
    uint32_t version;        /* Version of the Settings */
    uint32_t size;           /* Size of the Settings */
    uint32_t crc;            /* CRC of the Settings (computed with the crc initialized to 0) */
} __attribute__((__packed__)) settings_page_header_t;

typedef struct
{
    /* The header of the Seal */
    settings_page_header_t header;

    /* The Update Request flag */
    uint32_t update_requested;
} __attribute__((__packed__)) __attribute__ ((aligned (sizeof(uint32_t)))) settings_page_t ;

typedef struct
{
    bool is_initialized;

    /* FLASH */
    const settings_page_t * flash_settings_page;

    /* Settings */
    settings_page_t settings_page;

} bldr_settings_t;

static bldr_settings_t bldr_settings = { .is_initialized = false };

static inline bool _settings_page_init( bldr_settings_t * _bldr_settings );

static bool _settings_init( bldr_settings_t * p_bldr_settings )
{
    bool result = false;

    /* Set the flash_settings page */
    p_bldr_settings->flash_settings_page = (settings_page_t *)BOOTLOADER_SETTINGS_ADDRESS;

    /* Initialize the settings page */
    result = _settings_page_init( p_bldr_settings );
    EXIT_IF_FALSE( result, "_settings_page_init failed" );

    /* Set the initialized flag */
    p_bldr_settings->is_initialized = true;

_EXIT:
    return result;
}

static inline bool _settings_header_check( const settings_page_header_t * _header )
{
    /* Check the magic number, the settings version and its size */
    if( _header->version != BLDR_SETTINGS_VERSION ||
        _header->size != sizeof(settings_page_t) )
    {
        return false;
    }

    return true;
}

static inline uint32_t _settings_crc_calc( const settings_page_t * _settings_page )
{
    settings_page_t settings_page_temp = *_settings_page;
    uint32_t crc_calc;

    /* Nullify the settings_temp crc for the calculation purposes */
    settings_page_temp.header.crc = 0;

    /* Calculate the settings crc */
    crc_calc = dlcrc32_calculate_data( CRC_INITIAL_VALUE, (uint8_t *)&settings_page_temp, sizeof(settings_page_t) );

    return crc_calc;
}

static inline bool _settings_crc_check( const settings_page_t * _settings_page )
{
    uint32_t crc_calc;

    /* Calculate the settings crc */
    crc_calc = _settings_crc_calc( _settings_page );

    return ( crc_calc == _settings_page->header.crc ) ? true : false;
}

static bool _settings_validity_check( const settings_page_t * p_settings_page )
{
    bool result = false;

    /* Check the header first */
    result = _settings_header_check( &p_settings_page->header );
    EXIT_IF_FALSE( result, "_settings_header_check failed" );

    /* Now check the CRC of the settings */
    result = _settings_crc_check( p_settings_page );
    EXIT_IF_FALSE( result, "_settings_crc_check failed" );

_EXIT:
    return result;
}

static inline void _settings_page_init_default( settings_page_t * p_settings_page )
{
    /* Initialize the header */
    p_settings_page->header.version = BLDR_SETTINGS_VERSION;
    p_settings_page->header.size = sizeof( settings_page_t );
    p_settings_page->header.crc = 0;

    /* Initialize the values */
    p_settings_page->update_requested = false;
}

static inline bool _settings_page_init( bldr_settings_t * _bldr_settings )
{
    bool result = false;

    /* Check the validity of the settings page currently stored in the flash memory */
    result = _settings_validity_check( _bldr_settings->flash_settings_page );

    if( result == true )
    {
        /* Get the copy of the flash settings page */
        _bldr_settings->settings_page = *_bldr_settings->flash_settings_page;
    }
    else
    {
        /* Initialize the default settings page */
        _settings_page_init_default( &_bldr_settings->settings_page );
    }

    return true;
}

static void _settings_page_clear( bldr_settings_t * p_bldr_settings )
{
    nrf_nvmc_page_erase( (uint32_t)p_bldr_settings->flash_settings_page );
}

static inline bool _settings_page_compare( bldr_settings_t * p_bldr_settings )
{
    return ( memcmp( p_bldr_settings->flash_settings_page, &p_bldr_settings->settings_page, sizeof( settings_page_t )) == 0 ) ? true : false;
}

static bool _settings_page_save_blocking( bldr_settings_t * p_bldr_settings )
{
    /* Check if it is actually needed to update the flash */
    if( _settings_page_compare( p_bldr_settings ) == true )
    {
        return true;
    }

    /* First, clear the settings page */
    _settings_page_clear( p_bldr_settings );

    /* Calculate the CRC */
    p_bldr_settings->settings_page.header.crc = _settings_crc_calc( &p_bldr_settings->settings_page );

    /* Write the settings structure into the flash */
    nrf_nvmc_write_words( (uint32_t)p_bldr_settings->flash_settings_page, (uint32_t *)&p_bldr_settings->settings_page, WORD_CNT_GET(sizeof(settings_page_t)) );

    /* Check the settings page has been successfully written */
    if( _settings_page_compare( p_bldr_settings ) == false )
    {
        return false;
    }

    return true;
}

static void _settings_update_request( bldr_settings_t * p_bldr_settings, bool update_request )
{
    p_bldr_settings->settings_page.update_requested = update_request;
}

static bool _update_request( bldr_settings_t * p_bldr_settings )
{
    bool result = false;

    if( p_bldr_settings->is_initialized == false )
    {
        return false;
    }

    _settings_update_request( p_bldr_settings, true );

    result = _settings_page_save_blocking( p_bldr_settings );
    EXIT_IF_FALSE( result, "_settings_page_save_blocking failed" );

_EXIT:
    return result;
}

bool bldrUtil::init( void )
{
    return _settings_init( &bldr_settings );
}

bool bldrUtil::update_request( void )
{
    return _update_request( &bldr_settings );
}
