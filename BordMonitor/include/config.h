/***************************************************************************
 *                                                                         *
 *   (c) Art Tevs, MPI Informatik Saarbruecken                             *
 *       mailto: <tevs@mpi-sb.mpg.de>                                      *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#ifndef _CONFIG_H
#define	_CONFIG_H

#ifdef	__cplusplus
extern "C"
{
#endif

#include "customer.h"

// Version
#define VERSION_MAJOR 2
#define VERSION_MINOR 5
#define EE_CHECK_BYTE 'q'
#define VERSION_ADD_STR "on " __DATE__ " at " __TIME__ " for " CUSTOMER

#define DEVICE_CODING1 DEVID_11
#define DEVICE_CODING2 DEVID_12
#define DEVICE_CODING3 DEVID_13
#define HW_ID DEVID_1
    

#define USE_BM_LEDS_BIT       (0b00000100)
#define CARPC_INPUT_SET(a)    {g_deviceSettings.device_Settings1 &= 0b11100111; g_deviceSettings.device_Settings1 &= (a & 0x03);}
#define SUPPORT_SPECIAL_BIT   (0b00100000)

#define BACKCAM_INPUT()     (g_deviceSettings.device_Settings1 & 0b00000011)
#define USE_BM_LEDS()       (g_deviceSettings.device_Settings1 & 0b00000100)
#define CARPC_INPUT()      ((g_deviceSettings.device_Settings1 & 0b00011000) >> 3) // 0=radio, 1=cdchanger, 2=tape, 3=AUX
#define SUPPORT_SPECIAL()   (g_deviceSettings.device_Settings1 & 0b00100000)   // support for special features, like autom. close/open central lock
#define USE_PHOTOSENSOR()   (g_deviceSettings.device_Settings1 & 0b01000000)   // use photo sensor per default

// Settings (DEVID_12)
#define EMULATE_MID         (1 << 0)
#define RADIO_PROFESSIONAL  (1 << 1)
#define DSP_AMPLIFIER       (1 << 2)
#define EMULATE_BORDMONITOR (1 << 3)
#define REW_FF_ONMID        (1 << 4)
#define EMULATE_CDCHANGER   (1 << 5)
#define HAS_BACKCAM_SWITCH  ((DEVID_12 & (1 << 6)) == (1 << 6))
#define OPENBM_HW_1         ((DEVID_12 & (1 << 7)) == (1 << 7))

    
    
// Additional Device Settings (DEVID_13)
#define OBMS_AUT_CENTRALLOCK (1 << 0)
    
// -----------------------------------------------------------------------------
typedef struct _DeviceSettings
{
    // --------------------------------------
    // BMW settings
    // --------------------------------------
    uint8_t device_Settings1;
    uint8_t device_Settings2;
    
    // --------------------------------------
    // stuff
    // --------------------------------------
    uint8_t initSeed;
    uint8_t io_assignment[3];
    
}DeviceSettings;

extern DeviceSettings g_deviceSettings;
extern DeviceSettings g_deviceSettingsEEPROM;


#ifdef	__cplusplus
}
#endif

#endif	/* _CONFIG_H */

