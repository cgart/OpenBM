/*
* Copyright 2010-2013 Art Tevs <art@tevs.eu>
* This file is part of OpenBM (firmware).
*
* OpenBM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenBM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _CONFIG_H
#define	_CONFIG_H

#ifdef	__cplusplus
extern "C"
{
#endif

#include "customer.h"

// ---------------------------------------------------
// Configuration for coding of OpenBM 
// ---------------------------------------------------
// Version
#define VERSION_MAJOR 2
#define VERSION_MINOR 6
#define EE_CHECK_BYTE 'q'
#define VERSION_ADD_STR "on " __DATE__ " at " __TIME__ " for " CUSTOMER

#define DEVICE_CODING1 DEVID_11
#define DEVICE_CODING2 DEVID_12
#define DEVICE_CODING3 DEVID_13
#define HW_ID DEVID_1
    

// Settings (DEVID_11)
#define CARPC_INPUT_CD_BIT      (1 << 3)
#define CARPC_INPUT_TAPE_BIT    (1 << 4)
#define CARPC_INPUT_AUX_BIT     (CARPC_INPUT_CD_BIT | CARPC_INPUT_TAPE_BIT)

#define USE_BM_LEDS_BIT         (1 << 2)
#define SUPPORT_SPECIAL_BIT     (1 << 5) // support for special features, like autom. close/open central lock
#define USE_PHOTOSENSOR_BIT     (1 << 6) // use photo sensor per default
#define DISP_VLED_INV_BIT       (1 << 7) // the display LED is acting in inverse mode (default: VLED- = GND-> background light ON, inv: VLED- == GND -> background light OFF;)

    
// Settings (DEVID_12)
#define EMULATE_MID_BIT         (1 << 0)
#define RADIO_PROFESSIONAL_BIT  (1 << 1)
#define DSP_AMPLIFIER_BIT       (1 << 2)
#define EMULATE_BORDMONITOR_BIT (1 << 3)
#define REW_FF_ONMID_BIT        (1 << 4)
#define EMULATE_CDCHANGER_BIT   (1 << 5)
#define HAS_BACKCAMSWITCH_BIT   (1 << 6)
#define OPENBM_HW_1_REV_BIT     (1 << 7)  // @note - this only applies to first revision, green PCB


// Settings (DEVID_13)
#define OBMS_AUT_CENTRALLOCK_BIT (1 << 0) // lock/unlock centrallock automatically
#define DISP_BUTTONS_ON_IO_BIT   (1 << 7) // display buttons are mapped to the IO outputs instead of internal




// ---------------------------------------------------
// supporting macros
#define CARPC_INPUT()        ((g_deviceSettings.device_Settings1 &  0b00011000) >> 3) // 0=radio, 1=cdchanger, 2=tape, 3=AUX
#define CARPC_INPUT_SET(a)    {g_deviceSettings.device_Settings1 &= 0b11100111; g_deviceSettings.device_Settings1 &= (a & 0x03);}
#define BACKCAM_INPUT()     (g_deviceSettings.device_Settings1 & 0b00000011)  // 0,1,2,3 define as display output
#define USE_BM_LEDS()       (g_deviceSettings.device_Settings1 & USE_BM_LEDS_BIT)
#define SUPPORT_SPECIAL()   (g_deviceSettings.device_Settings1 & SUPPORT_SPECIAL_BIT)
#define USE_PHOTOSENSOR()   (g_deviceSettings.device_Settings1 & USE_PHOTOSENSOR_BIT) 
#define DISP_VLED_INV()     (g_deviceSettings.device_Settings1 & DISP_VLED_INV_BIT)   

#define EMULATE_MID()         (g_deviceSettings.device_Settings2 & EMULATE_BORDMONITOR_BIT)
#define RADIO_PROFESSIONAL()  (g_deviceSettings.device_Settings2 & RADIO_PROFESSIONAL_BIT)
#define DSP_AMPLIFIER()       (g_deviceSettings.device_Settings2 & DSP_AMPLIFIER_BIT)
#define EMULATE_BORDMONITOR() (g_deviceSettings.device_Settings2 & EMULATE_BORDMONITOR_BIT)
#define REW_FF_ONMID()        (g_deviceSettings.device_Settings2 & REW_FF_ONMID_BIT)
#define EMULATE_CDCHANGER()   (g_deviceSettings.device_Settings2 & EMULATE_CDCHANGER_BIT)
#define HAS_BACKCAM_SWITCH()  ((DEVID_12 & HAS_BACKCAMSWITCH_BIT) == HAS_BACKCAMSWITCH_BIT)
#define OPENBM_HW_1()         ((DEVID_12 & OPENBM_HW_1_REV_BIT) == OPENBM_HW_1_REV_BIT)

        
// Additional Device Settings (DEVID_13)
#define OBMS_AUT_CENTRALLOCK() (g_deviceSettings.device_Settings3 & (1 << 0))
#define DISP_BUTTONS_ON_IO()   (g_deviceSettings.device_Settings3 & (1 << 7))
    
// -----------------------------------------------------------------------------
typedef struct _DeviceSettings
{
    // --------------------------------------
    // BMW settings
    // --------------------------------------
    uint8_t device_Settings1;
    uint8_t device_Settings2;
    uint8_t device_Settings3;
    
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

