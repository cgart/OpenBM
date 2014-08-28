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

#ifndef _DISPLAY_H
#define	_DISPLAY_H

#include <base.h>

#ifdef	__cplusplus
extern "C"
{
#endif

#define DAC_I2C_ADDRESS 0b11000000
#define DISP_MOSFET_SETUP {DDRB |= (1 << DDB4); PORTB &= ~(1 << 4);}
#define DISP_MOSFET_OFF {PORTB &= ~(1 << 4);}
#define DISP_MOSFET_ON  {PORTB |= (1 << 4);}

// emualtion of buttons through IO (off is VCC, on is GND)
#define DISP_POWER_KEY_OVER_IO_ON() PORTB &= ~(1 << (5+0))
#define DISP_POWER_KEY_OVER_IO_OFF()  PORTB |= (1 << (5+0))

#define DISP_SWITCH_KEY_OVER_IO_ON() PORTB &= ~(1 << (5+1))
#define DISP_SWITCH_KEY_OVER_IO_OFF()  PORTB |= (1 << (5+1))

#define DISP_MENU_KEY_OVER_IO_ON() PORTB &= ~(1 << (5+0))
#define DISP_MENU_KEY_OVER_IO_OFF()  PORTB |= (1 << (5+0))
    
//------------------------------------------------------------------------------
// Init display, by loading data from eeprom
//------------------------------------------------------------------------------
extern void display_init(void);

//------------------------------------------------------------------------------
// Shut down display, this will store settings and turn it off
//------------------------------------------------------------------------------
extern void display_powerOn(void);
extern void display_powerOff(void);
extern void display_turnOff(void); // turn off display without saving the power state to the EEPROM
//extern void display_turnOn(void);  // turn on display without saving to EEPROM
extern void display_tryTurnOn(void); // turn on display if state in EEPROM is active

//------------------------------------------------------------------------------
// Toggle display power
//------------------------------------------------------------------------------
//extern void display_TogglePower(uint8_t writeToEeprom);

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
//extern void display_setPowerState(uint8_t state, bool writeToEeprom);
extern uint8_t display_getPowerState(void);

//------------------------------------------------------------------------------
// Set input state (0=vga, 1=av1, 2=av2)
//------------------------------------------------------------------------------
extern void display_setInputState(uint8_t state);
extern uint8_t display_getInputState(void);
extern void display_updateInputState(uint8_t state);

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


// Activate backup camera. This will switch a backup camera on the VGA driver bord
// Hence supported VGA driver board must be installed
extern void display_enableBackupCameraInput(uint8_t en);

#ifdef	__cplusplus
}
#endif

#endif	/* _DISPLAY_H */

