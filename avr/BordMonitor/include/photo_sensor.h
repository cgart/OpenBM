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
* PhotoSensor handling of OpenBM
*
* Created on August 22, 2011, 12:12 AM
*/

#ifndef _PHOTO_SENSOR_H
#define	_PHOTO_SENSOR_H

#ifdef	__cplusplus
extern "C"
{
#endif

extern uint8_t photoSensorRawValue;

extern void photo_init(void);
extern void photo_tick(void);
extern void photo_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen);
inline void photo_set_adc_raw_value(uint8_t adcval) {photoSensorRawValue = adcval;}
extern uint8_t photo_get_adc_raw_value(void);
extern void photo_setup_calibration(void);//uint8_t minCalib, uint8_t maxCalib, uint8_t minRange, uint8_t maxRange);
extern uint8_t photo_get_value(void);
extern void photo_enable(uint8_t flag);
extern uint8_t photo_use_state(void);

extern uint8_t photo_get_max_value(void);
extern uint8_t photo_get_min_value(void);
extern void photo_set_max_value(uint8_t);
extern void photo_set_min_value(uint8_t);
extern void photo_set_max_calib_value(uint8_t);
extern void photo_set_min_calib_value(uint8_t);
extern bool photo_is_bright_enough(void); // returns true if the value of photo-sensor is considered as bright, i.e. v-min/max-min > 0.5

#ifdef	__cplusplus
}
#endif

#endif	/* _PHOTO_SENSOR_H */

