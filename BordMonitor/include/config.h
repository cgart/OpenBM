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
#define VERSION_MINOR 0
#define VERSION_ADD_STR "[2.0.0-rc1] compiled on " __DATE__ " at " __TIME__

#define DEVICE_CODING1 DEVID_11
#define DEVICE_CODING2 DEVID_12

// Settings
#define RADIO_BUISINESS    (1 << 0)
#define RADIO_PROFESSIONAL (1 << 1)
#define DSP_AMPLIFIER      (1 << 2)
#define EMULATE_MID        (1 << 3)
#define REW_FF_ONMID       (1 << 4)
#define EMULATE_CDCHANGER  (1 << 5)
    
#define BACKCAM_INPUT()     (g_deviceSettings.device_Settings1 & 0b00000011)
#define USE_BM_LEDS()       (g_deviceSettings.device_Settings1 & 0b00000100)
#define CARPC_INPUT()      ((g_deviceSettings.device_Settings1 & 0b00011000) >> 3) // 0=radio, 1=cdchanger, 2=tape, 3=AUX
#define SUPPORT_SPECIAL()   (g_deviceSettings.device_Settings1 & 0b00100000)   // support for special features, like autom. close/open central lock

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

