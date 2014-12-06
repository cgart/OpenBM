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

#define PORT_EXPANDER_ENCODER_ADD 0b01000000
#define PORT_EXPANDER_RADIO_ADD   0b01000010
#define PORT_EXPANDER_BMBT_ADD    0b01000100

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
