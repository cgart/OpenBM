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

#define DAC_I2C_ADDRESS 0b11000000

//------------------------------------------------------------------------------
// Init display, by loading data from eeprom
//------------------------------------------------------------------------------
extern void display_init(void);

//------------------------------------------------------------------------------
// Shut down display, this will store settings and turn it off
//------------------------------------------------------------------------------
extern void display_shutDown(void);

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
extern uint8_t display_getPowerState(void);

//------------------------------------------------------------------------------
// Set input state (0=vga, 1=av1, 2=av2)
//------------------------------------------------------------------------------
extern void display_setInputState(uint8_t state);
extern uint8_t display_getInputState(void);

//------------------------------------------------------------------------------
// Set voltage levels for the power key and input key (12 bit)
// Voltage level is measured by the following equation:
//      Vkey = (5V * data) / 4096
//------------------------------------------------------------------------------
extern void display_setVoltagePower(uint16_t data);
extern void display_setVoltageSwitch(uint16_t data);

//------------------------------------------------------------------------------
// Get voltage currently used for the power and switch button
// Voltage level is measured by the following equation:
//      data = Vkey * 4096 / 5V
//------------------------------------------------------------------------------
extern uint16_t display_getVoltagePower(void);
extern uint16_t display_getVoltageSwitch(void);


//------------------------------------------------------------------------------
// Regulate background light brightness by specifying duty cycle
// 0   -> bg light off
// 100 -> bg light full on (linear scaling)
//------------------------------------------------------------------------------
extern void display_setBackgroundLight(uint8_t duty);
extern uint8_t display_getBackgroundLight(void);

#ifdef	__cplusplus
}
#endif

#endif	/* _DISPLAY_H */

