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
* This module checks for the power modes of the CPU: sleep, shutdown, ...
*
* Created on September 5, 2011, 10:24 AM
*/

#ifndef POWER_MODULE_H
#define	POWER_MODULE_H

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum _RunningMode
{
    INIT = 0,
    RUN = 1,
    STOP_IBUS = 2,
    PREPARE_SHUTDOWN = 6
}RunningMode;

extern void power_init(void);
extern void power_tick(void);
extern void power_stop(void);
extern void power_resume(void);
extern void power_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen);
extern void power_reset_cpu(void);

extern RunningMode power_get_running_mode(void);
extern void power_set_running_mode(RunningMode);

//! Prepare to shutdown. The main module will shutdown after 2 seconds
extern void power_prepare_shutdown(void);

extern void power_setHWIgnitionState(uint8_t state);
extern uint8_t power_getHWIgnitionState(void);

#ifdef	__cplusplus
}
#endif

#endif	/* POWER_MODULE_H */

