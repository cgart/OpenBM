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
#ifndef UART_LIB_H
#define UART_LIB_H

//--------------------------------------------------------------------------
// Include basic headers
//--------------------------------------------------------------------------
#include "base.h"

#define UART_USE_INTERFACE_0 1
#define UART_USE_INTERFACE_1 0

//--------------------------------------------------------------------------
// You have to setup these macros for your purpose
//--------------------------------------------------------------------------
/** Size of the circular receive buffer, must be power of 2 */
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE 256
#endif

/** Size of the circular transmit buffer, must be power of 2 */
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE 256
#endif

/** If we use external ram for the uart buffer **/
#ifdef UART_BUFFER_EXTERNAL_RAM
	//! Adress where to store the tx buffer in the external ram
	#define UART_TX_BUFFER_ADDR	0x1000

	//! Adress of the external ram in the rx buffer
	#define UART_RX_BUFFER_ADDR	0x1100
#endif

#if 1
//--------------------------------------------------------------------------
// Definitions for ATmega8
//--------------------------------------------------------------------------
#ifdef __AVR_ATmega8__
	#define MCU_USART_RXC_vect USART_RXC_vect
	#define MCU_RXCIE RXCIE
	#define MCU_TXCIE TXCIE
	#define MCU_UDR   UDR
	#define MCU_UBRRH UBRRH
	#define MCU_UBRRL UBRRL
	#define MCU_UCSRA UCSRA
	#define MCU_UCSRB UCSRB
	#define MCU_UCSRC UCSRC
	#define MCU_UDRE  UDRE
	#define MCU_RXC   RXC
	#define MCU_RXEN  RXEN
	#define MCU_TXEN  TXEN
	#define MCU_UCSZ0 UCSZ0
	#define MCU_UCSZ1 UCSZ1
	#define MCU_URSEL _BV(URSEL)
#endif

//--------------------------------------------------------------------------
// Definitions for ATmega32
//--------------------------------------------------------------------------
#ifdef __AVR_ATmega32__
	#define MCU_USART_RXC_vect USART_RXC_vect
	#define MCU_USART_TXC_vect USART_TXC_vect
	#define MCU_RXCIE RXCIE
	#define MCU_TXCIE TXCIE
	#define MCU_UDR   UDR
	#define MCU_UBRRH UBRRH
	#define MCU_UBRRL UBRRL
	#define MCU_UCSRA UCSRA
	#define MCU_UCSRB UCSRB
	#define MCU_UCSRC UCSRC
	#define MCU_UDRE  UDRE
	#define MCU_RXC   RXC
	#define MCU_RXEN  RXEN
	#define MCU_TXEN  TXEN
	#define MCU_UCSZ0 UCSZ0
	#define MCU_UCSZ1 UCSZ1
	#define MCU_URSEL _BV(URSEL)
#endif

//--------------------------------------------------------------------------
// Definitions for ATmega128
//--------------------------------------------------------------------------
#ifdef __AVR_ATmega128__
	#define MCU_USART_RXC_vect USART0_RX_vect
	#define MCU_RXCIE          RXCIE0
	#define MCU_UDR            UDR0
	#define MCU_UDR_0          UDR0
	#define MCU_UDR_1          UDR1
	#define MCU_UBRRH          UBRR0H
	#define MCU_UBRRL          UBRR0L
	#define MCU_UCSRA          UCSR0A
	#define MCU_UCSRB          UCSR0B
	#define MCU_UCSRC          UCSR0C
	#define MCU_UDRE           UDRE0
	#define MCU_RXC            RXC0
	#define MCU_RXEN           RXEN0
	#define MCU_TXEN           TXEN0
	#define MCU_UCSZ0          UCSZ00
	#define MCU_UCSZ1          UCSZ01
	#define MCU_URSEL          0
#endif

//--------------------------------------------------------------------------
// Definitions for AT90CAN128
//--------------------------------------------------------------------------
#ifdef __AVR_AT90CAN128__
	#define MCU_USART_RXC_vect USART0_RX_vect
	#define MCU_RXCIE RXCIE0
	#define MCU_UDR   UDR0
	#define MCU_UBRRH UBRR0H
	#define MCU_UBRRL UBRR0L
	#define MCU_UCSRA UCSR0A
	#define MCU_UCSRB UCSR0B
	#define MCU_UCSRC UCSR0C
	#define MCU_UDRE  UDRE0
	#define MCU_RXC   RXC0
	#define MCU_RXEN  RXEN0
	#define MCU_TXEN  TXEN0
	#define MCU_UCSZ0 UCSZ00
	#define MCU_UCSZ1 UCSZ01
	#define MCU_URSEL 0
#endif

//--------------------------------------------------------------------------
// Definitions for ATmega168
//--------------------------------------------------------------------------
#ifdef __AVR_ATmega168__
	#define MCU_USART_RXC_vect USART_RX_vect
	#define MCU_RXCIE RXCIE0
	#define MCU_UDR   UDR0
	#define MCU_UBRRH UBRR0H
	#define MCU_UBRRL UBRR0L
	#define MCU_UCSRA UCSR0A
	#define MCU_UCSRB UCSR0B
	#define MCU_UCSRC UCSR0C
	#define MCU_UDRE  UDRE0
	#define MCU_RXC   RXC0
	#define MCU_RXEN  RXEN0
	#define MCU_TXEN  TXEN0
	#define MCU_UCSZ0 UCSZ00
	#define MCU_UCSZ1 UCSZ01
	#define MCU_URSEL 0
#endif

//--------------------------------------------------------------------------
// Definitions for ATmega644
//--------------------------------------------------------------------------
#ifdef __AVR_ATmega644__
	#define MCU_USART_RXC_vect USART0_RX_vect
	#define MCU_RXCIE RXCIE0
	#define MCU_UDR   UDR0
	#define MCU_UBRRH UBRR0H
	#define MCU_UBRRL UBRR0L
	#define MCU_UCSRA UCSR0A
	#define MCU_UCSRB UCSR0B
	#define MCU_UCSRC UCSR0C
	#define MCU_UDRE  UDRE0
	#define MCU_RXC   RXC0
	#define MCU_RXEN  RXEN0
	#define MCU_TXEN  TXEN0
	#define MCU_UCSZ0 UCSZ00
	#define MCU_UCSZ1 UCSZ01
	#define MCU_URSEL 0
#endif
#endif


//--------------------------------------------------------------------------
// Check if avr does support more than one uart interface
//--------------------------------------------------------------------------
#if defined(__AVR_ATmega128__) || defined(__AVR_AT90CAN128__) ||  defined(__AVR_ATmega168__) || defined(__AVR_ATmega644__)
#define UART_SUPPORT_MULTIPLE_INTERFACE
#endif


/* test if the size of the circular buffers fits into SRAM */
#if ( (UART_RX_BUFFER_SIZE+UART_TX_BUFFER_SIZE) >= (RAMEND-0x60 ) )
#error "size of UART_RX_BUFFER_SIZE + UART_TX_BUFFER_SIZE larger than size of SRAM"
#endif

#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1)

#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
#endif

//--------------------------------------------------------------------------
// Useful macros
//--------------------------------------------------------------------------

//! Compute uart value based on magic formula (single speed mode)
#define UART_COMPUTE_BAUD(f_cpu, baudrate) ((f_cpu/(16L * baudrate)) - 1L)

//! Compute baud value based on magic formula (double speed mode)
#define UART_COMPUTE_BAUD_DX(f_cpu, baudrate) ((f_cpu/(8UL * baudrate)) - 1UL)

//--------------------------------------------------------------------------
// Check if more then one uart interface is in use
//--------------------------------------------------------------------------
//if no uart interface was specified, so just set all to 0 to force error
#ifndef UART_USE_INTERFACE_0
#define UART_USE_INTERFACE_0 0
#endif

#ifndef UART_USE_INTERFACE_1
#define UART_USE_INTERFACE_1 0
#endif

// Some of the uart interface must be specified
#if (UART_USE_INTERFACE_0 == 0) && (UART_USE_INTERFACE_1 == 0)
#	error "Non compatible settings in UART_USE_INTERFACE_0 and UART_USE_INTERFACE_1"
#endif

#include "uart.h"

//--------------------------------------------------------------------------
// Global function definitions
//--------------------------------------------------------------------------
#ifdef UART_SUPPORT_MULTIPLE_INTERFACE
	class Uart0;
	class Uart1;

	#if UART_USE_INTERFACE_0 == 1
		#include "uart0.h"
		//! Initialize interface 0 (@see UART_init() )
		extern Uart0* UART_init0(void);
	#endif

	#if UART_USE_INTERFACE_1 == 1
		#include "uart1.h"
		//! Initialize interface 1 (@see UART_init() )
		extern Uart1* UART_init1(void);
	#endif

		//! Just inline to be optimized
		//extern inline Uart0* UART_init(void) { return UART_init0(); }

#else
	class Uart0;

	#if UART_USE_INTERFACE_0 == 1
		#include "uart0.h"
		//! Initialize interface 0 (@see UART_init() )
		extern Uart0* UART_init0(void);
	#endif

	#include "uart.h"

	/**
	 * Initialize uart interface and return pointer to
	 * the class to drive the uart. The port is set per default
	 * to 9600 8N1 mode and RX/TX bits are active.
	 * @note Some avr does support more than one uart interface,
	 * so calling appropriate function (_init0, init1, ...)
	 * will force the class to use according uart ports.
	**/
	//extern Uart* UART_init(void);

#endif


#endif
