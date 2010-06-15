/* 
 * File:   display.h
 * Author: tevs
 *
 * Simple routine for OpenBM_E39 to take care of display build in.
 *
 * Created on June 1, 2010, 10:57 PM
 */

#ifndef _DISPLAY_H
#define	_DISPLAY_H

#include <base.h>

#ifdef	__cplusplus
extern "C"
{
#endif

//------------------------------------------------------------------------------
// Init display, by loading data from eeprom
//------------------------------------------------------------------------------
extern void display_init(void);

//------------------------------------------------------------------------------
// Toggle display power
//------------------------------------------------------------------------------
extern void display_TogglePower(uint8_t writeToEeprom);

//------------------------------------------------------------------------------
// Toggle display input
//------------------------------------------------------------------------------
extern void display_ToggleInput(uint8_t writeToEeprom);

//------------------------------------------------------------------------------
// update display buttons and status LEDs
// Logic to switch display on/off and to change inputs by bDISP button -------
//------------------------------------------------------------------------------
extern void display_updateState(void);

//------------------------------------------------------------------------------
// Set power state (0=off, 1=on)
//------------------------------------------------------------------------------
extern void display_setPowerState(uint8_t state);

//------------------------------------------------------------------------------
// Set input state (0=vga, 1=av1, 2=av2)
//------------------------------------------------------------------------------
extern void display_setInputState(uint8_t state);


#ifdef	__cplusplus
}
#endif

#endif	/* _DISPLAY_H */

