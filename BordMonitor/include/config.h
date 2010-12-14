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
#define VERSION_MAJOR 1
#define VERSION_MINOR 1
#define VERSION_ADD_STR "[beta1] compiled on " __DATE__ " at " __TIME__

#define DEVICE_CODING1 DEVID_11
#define DEVICE_CODING2 DEVID_12

// Settings
#define RADIO_BUISINESS    (1 << 0)
#define RADIO_PROFESSIONAL (1 << 1)
#define DSP_AMPLIFIER      (1 << 2)
#define EMULATE_MID        (1 << 3)
#define REW_FF_ONMID       (1 << 4)

#define USE_BM_LEDS()      ((g_deviceSettings.device_Settings1 & 4) == 4)
#define BACKCAM_INPUT()    (g_deviceSettings.device_Settings1 & 3)

// Get Deivec settings out of the device ID
#define DEVICE_DISP_IDLE   (((DEVID_5 << 8) | DEVID_6) & 0xFFF)
#define DEVICE_DISP_SWITCH (((DEVID_7 << 8) | DEVID_8) & 0xFFF)
#define DEVICE_DISP_POWER  (((DEVID_9 << 8) | DEVID_10) & 0xFFF)

// -----------------------------------------------------------------------------
typedef struct _DeviceSettings
{
    // --------------------------------------
    // BMW settings
    // --------------------------------------
    uint8_t device_Settings1;
    uint8_t device_Settings2;

    // --------------------------------------
    // Hardware Settings
    // --------------------------------------
    uint8_t photo_minValue;
    uint8_t photo_maxValue;
    
    // --------------------------------------
    // Display settings
    // --------------------------------------
    //uint16_t dac_maxVoltage;  // in DAC value 12bit (either 3.3V or 5V key)
    uint16_t dac_idleVoltage;   // voltage used when idle
    uint16_t dac_PowerKey;      // voltage when Power-Button pressed
    uint16_t dac_SwitchKey;     // voltage when button to switch input pressed
    
    // --------------------------------------
    // stuff
    // --------------------------------------
    uint8_t initSeed;

}DeviceSettings;

extern DeviceSettings g_deviceSettings;
extern DeviceSettings g_deviceSettingsEEPROM;

#ifdef	__cplusplus
}
#endif

#endif	/* _CONFIG_H */

