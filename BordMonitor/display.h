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
// Set eeprom value to the given state (use this, when reseting the state)
//------------------------------------------------------------------------------
extern void display_setPowerState(uint8_t state);

//------------------------------------------------------------------------------
// Toggle display input
//------------------------------------------------------------------------------
extern void display_ToggleInput(uint8_t writeToEeprom);

//------------------------------------------------------------------------------
// Set input state (use this when reset the state)
//------------------------------------------------------------------------------
extern void display_setInputState(uint8_t state);

//------------------------------------------------------------------------------
// update display buttons and status LEDs
// Logic to switch display on/off and to change inputs by bDISP button -------
//------------------------------------------------------------------------------
extern void display_updateState(void);



#ifdef	__cplusplus
}
#endif

#endif	/* _DISPLAY_H */

