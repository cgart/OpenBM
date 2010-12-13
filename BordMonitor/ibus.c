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
#include "ibus.h"
#include "uart.h"
#include "leds.h"

#define IBUS_USE_SHORT_BUFFER

#ifdef IBUS_USE_SHORT_BUFFER
    #define IBUS_MSG_TX_BUFFER_SIZE       256
    #define IBUS_MSG_RX_BUFFER_SIZE       256

    typedef uint8_t posptr_t;
    #define inc_posptr(ptr, mask) ptr++;
    #define inc_posptr_rx(ptr) ptr++;
    #define inc_posptr_tx(ptr) ptr++;
#else
    #define IBUS_MSG_TX_BUFFER_SIZE       256
    #define IBUS_MSG_RX_BUFFER_SIZE       256

    typedef uint16_t posptr_t;
    #define inc_posptr(ptr, mask) (ptr + 1) & mask;
    #define inc_posptr_rx(ptr) (ptr + 1) & IBUS_MSG_RX_BUFFER_SIZE_MASK;
    #define inc_posptr_tx(ptr) (ptr + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
#endif

#define IBUS_MSG_TX_BUFFER_SIZE_MASK  (IBUS_MSG_TX_BUFFER_SIZE - 1)
#define IBUS_MSG_RX_BUFFER_SIZE_MASK  (IBUS_MSG_RX_BUFFER_SIZE - 1)

uint8_t g_ibus_TxBuffer[IBUS_MSG_TX_BUFFER_SIZE];
uint8_t g_ibus_RxBuffer[IBUS_MSG_RX_BUFFER_SIZE];

void(*g_ibus_MsgCallback)(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen);

volatile uint8_t  g_ibus_State = IBUS_STATE_IDLE;
volatile posptr_t g_ibus_RxPos = 0;
volatile posptr_t g_ibus_RxLen = 2;
posptr_t g_ibus_TxReadPos = 0;
posptr_t g_ibus_TxWritePos = 0;


//--------------------------------------------------------------------------
// Transmit message over the bus. Message is readed from the given
// buffer position. While transmitting we check if bus is free and if
// same bit was recieved as transmitted. This ensures correct handling
// This method is time critical and interrupts will be disabled.
//--------------------------------------------------------------------------
posptr_t ibus_transmit_msg(posptr_t from, posptr_t len)
{
    posptr_t oldFrom = from;
    
    if (IBUS_SENSTA_VALUE()) return from;
    
    BEGIN_ATOMAR;
    {
        IBUS_TX_SETUP();
        
        while(len > 0)
        {
            // load byte to transmit
            uint8_t byte = g_ibus_TxBuffer[from];
            uint8_t par  = 0;

            // transmit start bit and check if collision happens
            bit_clear(IBUS_TX_PORT, IBUS_TX_PIN);
            IBUS_BAUD_DELAY();
            if (IBUS_SENSTA_VALUE()) goto collision;

            // transmit bitwise and compute parity bit
            // while transmitting check if collision was detected
            int8_t i;
            for (i = 0; i < 8; i++, byte>>=1)
            {
                par ^= (byte & 1);
                if (byte & 1)
                    bit_set(IBUS_TX_PORT, IBUS_TX_PIN);
                else
                    bit_clear(IBUS_TX_PORT, IBUS_TX_PIN);
                IBUS_BAUD_DELAY();
                if (IBUS_SENSTA_VALUE()) goto collision;
            }

            // send parity bit
            if (par & 1)
                bit_set(IBUS_TX_PORT, IBUS_TX_PIN);
            else
                bit_clear(IBUS_TX_PORT, IBUS_TX_PIN);

            IBUS_BAUD_DELAY();
            if (IBUS_SENSTA_VALUE()) goto collision;

            // send stop bit
            bit_set(IBUS_TX_PORT, IBUS_TX_PIN);
            IBUS_BAUD_DELAY();
            if (IBUS_SENSTA_VALUE()) goto collision;

            // update current pointers
            inc_posptr_tx(from);
            len--;
        }

        oldFrom = from;
    }


// return on failure
collision:

    END_ATOMAR;
    IBUS_TX_SETUP();
    return oldFrom;
}

//--------------------------------------------------------------------------
void ibus_setMessageCallback(void(*cb)(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen))
{
    g_ibus_MsgCallback = cb;
}

//--------------------------------------------------------------------------
uint8_t ibus_calcChecksum(uint8_t* pBuffer)
{
  if(pBuffer == NULL)
    return 0;

  posptr_t i;
  posptr_t len;
  uint8_t checksum;

  checksum = 0;
  len = pBuffer[1] + 1;

  for(i = 0; i < len;)
  {
      checksum ^= pBuffer[i];
      inc_posptr_tx(i);
  }

  return checksum;
}

//--------------------------------------------------------------------------
void ibus_recieveCallback(uint8_t c, uint8_t error)
{
    // check if there was an error, then reset state and wait until bus get free
    if (error != 0)
    {
        g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
        IBUS_TIMEOUT_RECEIVE_ERROR();
        return;
    }

    // go to recieve state and start timer which check for timeout
    g_ibus_State = IBUS_STATE_RECEIVING;
    TIFR1 |= (1 << TOV1);
    IBUS_TIMEOUT_RECEIVE();

    uint8_t msgReceived = 0;
    
    BEGIN_ATOMAR;
    {
        // put recieved byte into buffer
        g_ibus_RxBuffer[g_ibus_RxPos] = c;

        // notice length of the message
        if (g_ibus_RxPos == 1)
        {
            g_ibus_RxLen = c + 2;
        }

        inc_posptr_rx(g_ibus_RxPos);

        // if there were enough bytes recieved, then compare checksum
        if (g_ibus_RxPos >= g_ibus_RxLen)
        {
            IBUS_TIMER_DISABLE_INTERRUPT();

            g_ibus_RxPos = 0;
            g_ibus_RxLen = 2;

            uint8_t chk = ibus_calcChecksum(&g_ibus_RxBuffer[0]);
            msgReceived = (chk == c);

            g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
            IBUS_TIMEOUT_WAIT_FREE_BUS();
        }
    }
    END_ATOMAR;

    if (msgReceived && g_ibus_MsgCallback)
        g_ibus_MsgCallback(g_ibus_RxBuffer[0], g_ibus_RxBuffer[2], &g_ibus_RxBuffer[3], g_ibus_RxBuffer[1]-2);
}

//--------------------------------------------------------------------------
void ibus_uartReceiveCallback(void)
{
    while(1)
    {
        unsigned int data = uart_getc();
        if (data & UART_NO_DATA) break;
        ibus_recieveCallback(data & 0xFF, (data & 0xFF00) >> 8);
    }
}

//--------------------------------------------------------------------------
void ibus_sendMessage(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msgLength, uint8_t numberOfTries)
{
    // put number of retries
    g_ibus_TxBuffer[g_ibus_TxWritePos] = numberOfTries; inc_posptr_tx(g_ibus_TxWritePos);

    posptr_t oldWritePos = g_ibus_TxWritePos;
    uint8_t i;

    // put message into queue
    g_ibus_TxBuffer[g_ibus_TxWritePos] = src; inc_posptr_tx(g_ibus_TxWritePos);
    g_ibus_TxBuffer[g_ibus_TxWritePos] = msgLength + 2; inc_posptr_tx(g_ibus_TxWritePos);
    g_ibus_TxBuffer[g_ibus_TxWritePos] = dst; inc_posptr_tx(g_ibus_TxWritePos);
    for (i=0; i < msgLength; i++)
    {
        g_ibus_TxBuffer[g_ibus_TxWritePos] = msg[i]; inc_posptr_tx(g_ibus_TxWritePos);
    }
    g_ibus_TxBuffer[g_ibus_TxWritePos] = ibus_calcChecksum(&g_ibus_TxBuffer[oldWritePos]); inc_posptr_tx(g_ibus_TxWritePos);
}

//--------------------------------------------------------------------------
void ibus_tick()
{
    // if no data in the buffer or we are not idle, then do nothing
    if (g_ibus_TxReadPos == g_ibus_TxWritePos || g_ibus_State != IBUS_STATE_IDLE) return;

    // ok we are idle and we have data in the buffer, then first, wait for free buffer
    if (IBUS_SENSTA_VALUE())
    {
        g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
        IBUS_TIMEOUT_WAIT_FREE_BUS();
        return;
    }
    
    // ok bus is free, we can submit a message
    posptr_t tryCounterPos = g_ibus_TxReadPos;
    int8_t numberOfTries = g_ibus_TxBuffer[g_ibus_TxReadPos]; inc_posptr_tx(g_ibus_TxReadPos);
    posptr_t len = (posptr_t)g_ibus_TxBuffer[(g_ibus_TxReadPos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK];

    // transmit message
    uart_setTxRx(0,0);
    posptr_t oldTxPos = g_ibus_TxReadPos;
    posptr_t newTxPos = ibus_transmit_msg(g_ibus_TxReadPos,  len + 2);

    g_ibus_State = IBUS_STATE_TRANSMITTING;
    
    BEGIN_ATOMAR;
    {
        // if there was a collision, then resend message if we still have trials
        if (newTxPos == oldTxPos)
        {
            numberOfTries--;
            if (numberOfTries >= 0)
            {
                g_ibus_TxBuffer[tryCounterPos] = numberOfTries;
                g_ibus_TxReadPos = tryCounterPos;

                g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
                IBUS_TIMEOUT_COLLISION();

                END_ATOMAR;
                return;
            }
            newTxPos = (posptr_t)(oldTxPos + len + 2) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
        }

        // ok message was either submitted successfully or message trials
        // counter is elapsed, we flush tx buffer and continue
        g_ibus_TxReadPos = newTxPos;
    }
    END_ATOMAR;

    g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
    IBUS_TIMEOUT_AFTER_TRANSMIT();
}

//--------------------------------------------------------------------------
void ibus_init()
{
    // initialize uart interface used for IBus communication
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
    uart_setFormat(8,1,1);
    uart_setTxRx(0,0);
    uart_setTransmitDoneCallback(NULL);
    uart_setReceiveCallback(ibus_uartReceiveCallback);

    BEGIN_ATOMAR;
    {
        // per default we do not use any uart interface
        g_ibus_TxReadPos = 0;
        g_ibus_TxWritePos = 0;
        g_ibus_RxPos = 0;
        g_ibus_RxLen = 2;
        g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;

        IBUS_SENSTA_SETUP();
        IBUS_TIMER_SETUP();
        IBUS_TIMEOUT_WAIT_FREE_BUS();

        // interrupt on falling edge on SEN/STA pin
        EICRA &= ~(1 << ISC00);
        EICRA |=  (1 << ISC01);
    }
    END_ATOMAR;
}

//--------------------------------------------------------------------------
// This timer is a wait timer, so the execution of IBus protocl is stopped
// while timer is running
//--------------------------------------------------------------------------
ISR(IBUS_TIMER_INTERRUPT)
{
    // disable timer interrupts
    IBUS_TIMER_DISABLE_INTERRUPT();

    // ok we have not to wait for bus
    g_ibus_State = IBUS_STATE_IDLE;
    g_ibus_RxPos = 0;
    g_ibus_RxLen = 2;

    // enable reciever, disable transmitter
    uart_flush();
    uart_setTxRx(0,1);
}

//--------------------------------------------------------------------------
// When SEN/STA down, then trigger this interrupt
//--------------------------------------------------------------------------
ISR(INT0_vect)
{
    if (g_ibus_State == IBUS_STATE_TRANSMITTING) return;
    
    g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
    IBUS_TIMEOUT_WAIT_FREE_BUS();
}