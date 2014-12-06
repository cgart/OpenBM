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


#ifndef UART_H
#define UART_H

 #ifdef __cplusplus
 extern "C" {
 #endif


/** @brief  UART Baudrate Expression
 *  @param  xtalcpu  system clock in Mhz, e.g. 4000000L for 4Mhz          
 *  @param  baudrate baudrate in bps, e.g. 1200, 2400, 9600     
 */
#define UART_BAUD_SELECT(baudRate,xtalCpu) ((xtalCpu)/((baudRate)*16l)-1)

/** @brief  UART Baudrate Expression for ATmega double speed mode
 *  @param  xtalcpu  system clock in Mhz, e.g. 4000000L for 4Mhz           
 *  @param  baudrate baudrate in bps, e.g. 1200, 2400, 9600     
 */
#define UART_BAUD_SELECT_DOUBLE_SPEED(baudRate,xtalCpu) (((xtalCpu)/((baudRate)*8l)-1)|0x8000)


/** Size of the circular receive buffer, must be power of 2 */
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE 256
#endif
/** Size of the circular transmit buffer, must be power of 2 */
//#ifndef UART_TX_BUFFER_SIZE
//#define UART_TX_BUFFER_SIZE 64
//#endif

/* test if the size of the circular buffers fits into SRAM */
//#if ( (UART_RX_BUFFER_SIZE+UART_TX_BUFFER_SIZE) >= (RAMEND-0x60 ) )
//#error "size of UART_RX_BUFFER_SIZE + UART_TX_BUFFER_SIZE larger than size of SRAM"
//#endif

/* 
** high byte error return code of uart_getc()
*/
#define UART_FRAME_ERROR      0x10              /* Framing Error by UART       */
#define UART_OVERRUN_ERROR    0x08              /* Overrun condition by UART   */
#define UART_PARENTY_ERROR    0x04              /* Overrun condition by UART   */
#define UART_BUFFER_OVERFLOW  0x02              /* receive ringbuffer overflow */
#define UART_NO_DATA          0x01              /* no receive data available   */


/*
** function prototypes
*/

/**
   @brief   Initialize UART and set baudrate 
   @param   baudrate Specify baudrate using macro UART_BAUD_SELECT()
   @return  none
*/
extern void uart_init(unsigned int baudrate);


/**
 *  @brief   Get received byte from ringbuffer
 *
 * Returns in the lower byte the received character and in the 
 * higher byte the last receive error.
 * UART_NO_DATA is returned when no data is available.
 *
 *  @param   void
 *  @return  lower byte:  received byte from ringbuffer
 *  @return  higher byte: last receive status
 *           - \b 0 successfully received data from UART
 *           - \b UART_NO_DATA           
 *             <br>no receive data available
 *           - \b UART_BUFFER_OVERFLOW   
 *             <br>Receive ringbuffer overflow.
 *             We are not reading the receive buffer fast enough, 
 *             one or more received character have been dropped 
 *           - \b UART_OVERRUN_ERROR     
 *             <br>Overrun condition by UART.
 *             A character already present in the UART UDR register was 
 *             not read by the interrupt handler before the next character arrived,
 *             one or more received characters have been dropped.
 *           - \b UART_FRAME_ERROR       
 *             <br>Framing Error by UART
 */
extern unsigned char uart_getc(void);
extern unsigned char uart_lastc(void);

extern uint8_t uart_last_error(void);
extern void uart_clear_error(void);

/**
 *  @brief   Return number of bytes waiting in the receive buffer
 *  @param   none
 *  @return  1 if there are bytes waiting in the receive buffer
 */
extern uint8_t uart_available(void);

/**
 *  @brief   Flush bytes waiting in receive buffer
 *  @param   none
 *  @return  none
 */
extern void uart_flush(uint8_t flushInternalFifo);

/**
 * @brief              Set frame format for transmitted data
 * @param wordLength   length of the word
 * @param numStopBits  number of stoppbits (0 or 1)
 * @param parity       Parity bit (0=no partity, 1=even, 2=odd)
 **/
extern void uart_setFormat(unsigned char wordLength, unsigned char numStopBits, unsigned char parity);

/**
 * @brief              Enable tx and/or rx unit
 **/
//extern void uart_setTxRx(unsigned char tx, unsigned char rx);
extern void uart_setRx(unsigned char rx);


 #ifdef __cplusplus
 }
 #endif

#endif // UART_H 

