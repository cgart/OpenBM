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
#ifndef UART_CLASS_0_H
#define UART_CLASS_0_H

#include "uart.h"

#if UART_USE_INTERFACE_0 == 1

#ifdef UART_SUPPORT_MULTIPLE_INTERFACE
#	if defined(__AVR_ATmega161__)
#		define ISR_UART0_TX_vect UART0_TX_vect
#		define ISR_UART0_RX_vect UART0_RX_vect
#	elif defined(__AVR_ATmega162__)
#		define ISR_UART0_TX_vect USART0_TXC_vect
#		define ISR_UART0_RX_vect USART0_RXC_vect
#	else
#		define ISR_UART0_TX_vect USART0_TX_vect
#		define ISR_UART0_RX_vect USART0_RX_vect
#	endif
#endif

/**
 * Derived class from Uart. Here Uart0 specific functions
 * are defined.
 **/
class Uart0 : public Uart {
public:

	/**
	 * Get a pointer to the singleton instance.
	 * Also initialize the uart interface wth appropriate data.
	 * You have to call this method first!
	 **/
	static Uart0* initialize();

	//! Singleton storing the uart0 interface
	static Uart0 mSingleton;

	/**
	 * Enable/disable transmission and receive flags
	 * @param bTX set to 1 to enable and to 0 to disable transmission
	 * @param bRX set to 1 to enable and to 0 to disable receive
	 **/
	inline void setTxRxFlags(uint8_t bTX, uint8_t bRX);

};

//--------------------------------------------------------------------------
void Uart0::setTxRxFlags(uint8_t bTX, uint8_t bRX)
{
	// activate flags
	*mUCSRB |= (bTX << MCU_TXEN) | (bRX << MCU_RXEN);

	// deactivate flags
	*mUCSRB &= ~((uint8_t)!bTX << MCU_TXEN);
	*mUCSRB &= ~((uint8_t)!bRX << MCU_RXEN);
}


#endif

#endif
