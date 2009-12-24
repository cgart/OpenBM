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
#ifndef UART_CLASS_H
#define UART_CLASS_H

#include "UartLib.h"

/**
 * Uart class responsable for communication through uart/usart interface.
 * To create an instance of the class call UART_init(). There is no
 * constructor and destructor, to reduce code size. We are using
 * c++ class to be able to combine all useful function in some module.
 *
 **/
class Uart {
public:

	/**
	 * Set uart baudrate. The according
	 * baud value will be computed
	 * @param baudrate 32Bit baudrate value (i.e. 115200)
	 **/
	inline void setBaudrate(uint32_t baudrate);

	/**
	 * Set baud directly without computing it. Check your avr datasheets
	 * to set this value correctly.
	 * @param baud Set baud value according to your datasheets
	 * @note You can use UART_COMPUTE_BAUD and UART_COMPUTE_BAUD_DX
	 * to let compiler compute the values for you.
	 **/
	inline void setBaud(uint16_t baud);

	/**
	 * Set callback function to be called when an input is there.
	 * Using the callback method you do not have to poll the uart
	 * interface for inputs, they will be just delivered to you.
	 * @param rxFunc Function pointer of the callback function
	 **/
	inline void setReceiveCallback(void (*rxFunc)(uint8_t c));

	/**
	 * Send one byte through the uart interface. This function
	 * locks until byte is sent. So be warned before use it
	 * @param byte Byte to be sent
	 **/
	inline void sendByteImmediate(uint8_t byte);

	/**
	 * Send byte through a buffered interface. The function
	 * will add the byte into the transmit buffer (FIFO) and will
	 * return immediately. As soon as the uart interface is
	 * free the byte will be send.
	 * @param byte Byte to be sent
	 **/
	inline void sendByte(uint8_t byte);

	/**
	 * Print a null-terminated string to the uart interface
	 **/
	inline void print(const char* str);

	/**
	 * Print a null terminated string from the programm memory.
	 **/
	inline void printp(const char* str);

	/**
	 * Check if receive buffer has valid data. Returns 0 if
	 * no data available. Otherwise returns the number of
	 * bytes in the buffer.
	 **/
	inline uint8_t hasData(void) { return isRxReady; }

	/**
	 * Get data. If there is data in the buffer, so it
	 * will be readed from there. If there is no data
	 * in the buffer, so the function will block until
	 * there is data available.
	 * @Note Be sure to check if there is
	 * data in the buffer (see hasData() )
	 **/
	inline uint8_t getByte();

	//! Functon pointer for the callback function
	void (*volatile mRxCallback)(uint8_t c);

	//! Check if transmit interface is ready
	volatile uint8_t isTxReady;

	//! Check if receive buffer is empty
	volatile uint8_t isRxReady;

	//! Circular buffer holding the bytes to transmit
	volatile uint8_t* mTxBuffer;

	//! Circular buffer to hold received data
	volatile uint8_t* mRxBuffer;

	//! Current position in the tx buffer
	volatile uint8_t mTxCurrentWritePos;

	//! Current position in the rx buffer
	volatile uint8_t mRxCurrentWritePos;

	//! Current read pos of the tx buffer
	volatile uint8_t mTxCurrentReadPos;

	//! Current read pos of the rx buffer
	volatile uint8_t mRxCurrentReadPos;

	//! UCSRA Register. If not support this is equal 0 (see datasheets of your avr)
	volatile uint8_t* mUCSRA;

	//! UCSRB Register. If not support this is equal 0 (see datasheets of your avr)
	volatile uint8_t* mUCSRB;

	//! UCSRC Register. If not support this is equal 0 (see datasheets of your avr)
	volatile uint8_t* mUCSRC;

	//! Baud rate register low
	volatile uint8_t* mUBRRL;

	//! Baud rate register high
	volatile uint8_t* mUBRRH;

	//! UDR Register
	volatile uint8_t* mUDR;
};

//--------------------------------------------------------------------------
void Uart::sendByte(uint8_t byte)
{
	// the interface is blocked from here
	uint8_t txready = isTxReady;
	isTxReady = 0;

	// if the inteface is free, then send immediately
	if (txready)
	{
		*mUDR = byte;
		return;
	}

	// writing to buffer must be done synchronously
	BEGIN_ATOMAR;
		// work on local variables
		uint8_t pos = mTxCurrentWritePos;

		// store the input in the buffer
		mTxBuffer[pos] = byte;
		mTxCurrentWritePos  = (pos + 1) & UART_TX_BUFFER_MASK;
	END_ATOMAR;
}

//--------------------------------------------------------------------------
void Uart::sendByteImmediate(uint8_t byte)
{
	while(isTxReady == 0);
	*mUDR = byte;
	isTxReady = 0;
}

//--------------------------------------------------------------------------
void Uart::setReceiveCallback(void (*rxFunc)(uint8_t c))
{
	mRxCallback = rxFunc;
}

//--------------------------------------------------------------------------
void Uart::setBaudrate(uint32_t baudrate)
{
	// compute according baudrate value and set appropriate bits
	uint16_t div = UART_COMPUTE_BAUD(F_CPU, baudrate);
	*mUBRRL = div;
	*mUBRRH = div >> 8;
}

//--------------------------------------------------------------------------
void Uart::setBaud(uint16_t baud)
{
	*mUBRRL = baud & 0xFF;
	*mUBRRH = baud >> 8;
}


//--------------------------------------------------------------------------
uint8_t Uart::getByte()
{
	// lock if there is no data in the buffer
	while (isRxReady == 0);

	// read data and set new pointer position
	uint8_t rpos = mRxCurrentReadPos;
	uint8_t data = mRxBuffer[rpos];
	mRxCurrentReadPos = (rpos + 1) & UART_RX_BUFFER_MASK;

	// this section can not be interrupted anymore,
	// because otherwise the value will not be counted properly
	// decrease the counter of bytes in the buffer
	BEGIN_ATOMAR;
		isRxReady--;
	END_ATOMAR;

	// return data back
	return data;
}

//--------------------------------------------------------------------------
void Uart::print(const char* str)
{
	register char c;

	// print until there is no more characters available
	while ( (c = *str++) ) {
		sendByte(c);
	}
}

//--------------------------------------------------------------------------
void Uart::printp(const char* progmem_str)
{
	register char c;

	// print until there is no more characters available
	while ( (c = pgm_read_byte(progmem_str++)) ) {
		sendByte(c);
	}
}

#endif
