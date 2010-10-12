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
#ifndef BASE_H
#define BASE_H

//--------------------------------------------------------------------------
// Define basic macros
//--------------------------------------------------------------------------
#ifndef F_CPU
	#define F_CPU 14745650UL
#endif
#define SCL_CLOCK 100000 // 20kHz for the I2C Bus


//--------------------------------------------------------------------------
// Include basic headers
//--------------------------------------------------------------------------
#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>


#define BEGIN_ATOMAR unsigned char _sreg = SREG; cli();
#define END_ATOMAR SREG = _sreg;
#define parity_odd_bit(v) parity_even_bit(v)^0x01
#define is_bit_set(a,b) ((a >> b) & 1)
#define bit_set(a,b)   a|=(1<<b)   // sbi
#define bit_clear(a,b) a&=~(1<<b)  // cbi


//--------------------------------------------------------------------------
// Global ticks
//--------------------------------------------------------------------------
#define TICKS_PER_SECOND() 80  // how much of task ticks in one second
#define TICKS_PER_QUARTERSECOND() 20
#define TICKS_PER_HALFSECOND() 40
#define TICKS_PER_TWO_SECONDS() 160
#define TICKS_PER_ONE_AND_A_HALF_SECONDS() 120
//#define TICK_INTERRUPT TIMER2_COMP_vect

// current number of ticks since last reset
typedef uint32_t ticks_t;
extern ticks_t g_tickNumber;

// init tick counter
//#define tick_init() {TIMSK |= (1 << OCIE2); TCCR2 = (1 << CS22) | (1 << CS21) | (1 << CS20);}
#define tick_init() {TCCR2 = (1 << CS22) | (1 << CS21) | (1 << CS20);}

// get current number of ticks
#define tick_get() g_tickNumber

// do ticking
//#define tick() {g_tickNumber++; TCNT2 = 0; OCR2 = 180;}
#define tick() {g_tickNumber++; TCNT2 = 0;}

// check if there is a tick
#define tick_event() (TCNT2 >= 180)

//--------------------------------------------------------------------------
// Definitions for different MCUs
//--------------------------------------------------------------------------
#ifdef __AVR_ATmega8__
	#define MCU_IVSELREG GICR
	#define MCU_COUNTERTYPE uint8_t
#endif

#ifdef __AVR_ATmega32__
	#define MCU_IVSELREG GICR
	#define MCU_COUNTERTYPE uint8_t
#endif

#ifdef __AVR_ATmega128__
	#define MCU_IVSELREG       MCUCR
	#define MCU_COUNTERTYPE    uint16_t
#endif

#ifdef __AVR_AT90CAN128__
	#define MCU_IVSELREG MCUCR
	#define MCU_COUNTERTYPE uint16_t
#endif

#ifdef __AVR_ATmega168__
	#define MCU_IVSELREG MCUCR
	#define MCU_COUNTERTYPE uint8_t
#endif

#ifdef __AVR_ATmega644__
	#define MCU_USART_RXC_vect USART0_RX_vect
	#define MCU_COUNTERTYPE uint8_t
#endif

#endif
