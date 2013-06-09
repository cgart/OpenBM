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

#ifndef BASE_H
#define BASE_H


//--------------------------------------------------------------------------
// Define basic macros
//--------------------------------------------------------------------------
#ifndef F_CPU
	#define F_CPU 14745650UL
#endif
#define SCL_CLOCK 100000L // 100kHz for the I2C Bus

//--------------------------------------------------------------------------
// Include basic headers
//--------------------------------------------------------------------------
#include <avr/io.h>
#include <inttypes.h>
#define __DELAY_BACKWARD_COMPATIBLE__
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
#define nop() asm volatile ("nop")

#define false 0
#define true 1
typedef uint8_t bool;

//--------------------------------------------------------------------------
// Global ticks
//--------------------------------------------------------------------------
#define TICKS_PER_SECOND 32  // how much of task ticks in one second
#define TICKS_PER_QUARTERSECOND 8
#define TICKS_PER_HALFSECOND 16
#define TICKS_PER_TWO_SECONDS 64
#define TICKS_PER_ONE_AND_A_HALF_SECONDS 48
#define TICKS_PER_ONE_EIGHTH_SECOND 4
#define TICKS_PER_X_SECONDS(a) (TICKS_PER_SECOND * a)

// current number of ticks since last reset
typedef uint32_t ticks_t;

extern ticks_t g_tickNumber;
extern uint8_t g_tickEventHappened;

// init tick counter
#define tick_reset() {g_tickNumber = 0; TCNT1=0; }
#define tick_init() {TCCR1A = 0; TIMSK1 |= (1 << OCIE1B); TCCR1B = (1 << CS12) | (1 << CS10); OCR1B=450; tick_reset(); }

// get current number of ticks
#define tick_get() g_tickNumber

// do ticking
#define tick() {g_tickNumber++; TCNT1 = 0; OCR1B = 450; g_tickEventHappened = 0;}

// check if there is a tick
#define tick_event() (g_tickEventHappened == 1)
#define tick_interrupt() ISR(TIMER1_COMPB_vect)

// ----------------------------------------------------------------------------
// Firmware/Hardware information struct
// ----------------------------------------------------------------------------
typedef struct
{
	uint8_t dev_key[12];            // current device key (unique !!!)
	uint16_t app_version;           // currently flashed version
	uint16_t crc16;                 // full checksum of the current program (not crypted)
        uint16_t prognum;               // how often the flash was already programmed
} bootldrinfo_t;

// Check whenever we can fit the bootloader info into one flash page
#define CHECK3(x, line) typedef char check ## line[(x) ? 1 : -1];
#define CHECK2(x, line)  CHECK3(x, line)
#define CHECK_SIZE(x)  CHECK2(x, __LINE__)
CHECK_SIZE(sizeof(bootldrinfo_t) <= SPM_PAGESIZE)

// check that xtea block size can fit exactly into the page size
CHECK_SIZE(SPM_PAGESIZE % 8 == 0)

// currently we only support small page sizes
CHECK_SIZE(SPM_PAGESIZE < 256);

// Short loading of two bytes into 16 bit word
#define HILO(data,hi,lo) {\
            ((uint8_t*)(&data))[0] = lo; \
            ((uint8_t*)(&data))[1] = hi; \
        }

//#define LAST_PAGE (BOOTLOADERSTARTADR - SPM_PAGESIZE)
#define LAST_PAGE ((BOOTLOADERSTARTADR / SPM_PAGESIZE) - 1)


#endif
