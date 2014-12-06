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
*
* Emulation of MID interface
* 
* Created on June 4, 2010, 12:12 AM
*/

#ifndef _EMUL_MID_H
#define	_EMUL_MID_H

#ifdef	__cplusplus
extern "C"
{
#endif


/**
 * Initialize emulation of the MID device
 **/
extern void mid_init(void);

/**
 * Run MID emulation task. This will check for buttons and execute corresponding actions
 **/
extern void mid_tick(void);

/**
 * React on messages
 **/
extern void mid_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen);

extern void mid_stop(void);
extern void mid_resume(void);

extern void mid_setEmulation(uint8_t mid, uint8_t bmbt);

#ifdef	__cplusplus
}
#endif

#endif	/* _EMUL_MID_H */

