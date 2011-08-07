/* 
 * File:   leds.h
 * Author: tevs
 *
 * Take care of OpenBM_E39 leds
 * 
 * Created on June 1, 2010, 11:02 PM
 */

#ifndef _LEDS_H
#define	_LEDS_H

#include <base.h>


#ifdef	__cplusplus
extern "C"
{
#endif

/**
 * Setup leds, this will enable corresponding output pins
 **/
extern void led_init(void);

/**
 * Tick LED task, this will turn on/off led according to their state
 **/
extern void led_tick(void);

// set led state (state can be any pattern, updated every 1/8 sec)
extern void led_red_set(uint8_t state);
extern void led_green_set(uint8_t state);
extern void led_yellow_set(uint8_t state);
extern void led_fan_set(uint8_t state);
extern void led_radio_set(uint8_t state);

// set direct state of led (just turn on/off led)
extern void led_red_immediate_set(uint8_t state);
extern void led_green_immediate_set(uint8_t state);
extern void led_yellow_immediate_set(uint8_t state);
extern void led_fan_immediate_set(uint8_t state);
extern void led_radio_immediate_set(uint8_t state);

extern void led_radioBlinkLock(uint8_t times);
extern void led_greenBlinkLock(uint8_t times);

#ifdef	__cplusplus
}
#endif

#endif	/* _LEDS_H */

