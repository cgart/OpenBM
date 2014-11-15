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

#include "uart0.h"

#if UART_USE_INTERFACE_0 == 1

Uart0 Uart0::mSingleton;

#ifndef UART_BUFFERS_EXTERNAL_RAM
	static uint8_t mUartRxBuffer[UART_RX_BUFFER_SIZE];
	static uint8_t mUartTxBuffer[UART_TX_BUFFER_SIZE];
#endif

//--------------------------------------------------------------------------
Uart0* Uart0::initialize()
{
	// intialize pointer and clear the structure
	Uart0* ptr = &Uart0::mSingleton;

	// setup port pointers
	ptr->mUCSRA = &MCU_UCSRA;
	ptr->mUCSRB = &MCU_UCSRB;
	ptr->mUCSRC = &MCU_UCSRC;
	ptr->mUBRRL = &MCU_UBRRL;
	ptr->mUBRRH = &MCU_UBRRH;
	ptr->mUDR   = &MCU_UDR;

	// setup default baudrate values
	*ptr->mUBRRL = UART_COMPUTE_BAUD(F_CPU, 9600);
	*ptr->mUBRRH = UART_COMPUTE_BAUD(F_CPU, 9600) >> 8;

	// setup 8N1 per default
	*(ptr->mUCSRB) = (1 << MCU_TXEN) | (1 << MCU_RXEN) | (1 << MCU_TXCIE) | (1 << MCU_RXCIE);

	#ifdef URSEL0
	*(ptr->mUCSRC) = (1 << URSEL0) | (1 << UCSZ00) | (1 << UCSZ01);
	#else
	*(ptr->mUCSRC) = (1 << MCU_UCSZ0) | (1 << MCU_UCSZ1);
	#endif

	// setup buffer pointers
#ifndef UART_BUFFERS_EXTERNAL_RAM
	ptr->mTxBuffer = mUartTxBuffer;
	ptr->mRxBuffer = mUartRxBuffer;
#else
	ptr->mTxBuffer = UART_TX_BUFFER_ADDR;
	ptr->mRxBuffer = UART_TX_BUFFER_ADDR;
#endif

	// first wait until interface is ready to use
	ptr->isTxReady = 1;
	ptr->isRxReady = 0;

	// enable interrupts, because our code is interrupt driven
	sei();

	// return pointer back
	return ptr;
}


//--------------------------------------------------------------------------
ISR(MCU_USART_RXC_vect)
{
	static Uart0* ptr = &Uart0::mSingleton;
	uint8_t sreg = SREG;

	// get data from the input
	uint8_t data = *ptr->mUDR;

	// work on local variables (produce faster code)
	void (* callback)(uint8_t c) = ptr->mRxCallback;

	// do we have callback method
	if (callback)
	{
		// call the callback
		callback(data);
	}else{
		// get current pos and number of valid bytes in buffer
		register uint8_t pos = ptr->mRxCurrentWritePos;
		register uint8_t counter = ptr->isRxReady;

		// store the input in the buffer
		ptr->mRxBuffer[pos] = data;
		ptr->mRxCurrentWritePos = (pos+1) & UART_RX_BUFFER_MASK;

		// we have now more valid bytes in the buffer
		counter++;
		ptr->isRxReady = counter;
	}

	// reset the sreg register
	SREG = sreg;
}

//--------------------------------------------------------------------------
ISR(MCU_USART_TXC_vect)
{
	static Uart0* ptr = &Uart0::mSingleton;

	uint8_t sreg = SREG;

	// setup local variables for faster code
	uint8_t wpos = ptr->mTxCurrentWritePos;
	uint8_t rpos = ptr->mTxCurrentReadPos;

	// set the interface as ready
	ptr->isTxReady = 1;

	// check if we have data in the tx buffer
	if (wpos != rpos)
	{
		// send data from the buffer
		ptr->mTxCurrentReadPos = ( rpos + 1 ) & UART_TX_BUFFER_MASK;
		*(ptr->mUDR) = ptr->mTxBuffer[rpos];
	}

	SREG = sreg;
}


#endif
