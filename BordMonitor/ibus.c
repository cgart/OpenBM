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

uint8_t  g_ibus_State = IBUS_STATE_IDLE;
posptr_t g_ibus_TxReadPos_old = 0;
posptr_t g_ibus_TxReadPos = 0;
posptr_t g_ibus_TxWritePos = 0;
posptr_t g_ibus_RxPos = 0;
posptr_t  g_ibus_RxLen = 0;

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
void ibus_transmitCallback(void)
{
    PORTD &= ~(1 << 6);

    BEGIN_ATOMAR;
    {
        // disable collision interrupt and
        IBUS_SENSTA_DISABLE_INTERRUPT();
        uart_setTransmitDoneCallback(NULL);
        
        // start to wait for free bus, this will introduce small delay between msgs
        g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
        IBUS_TIMEOUT_AFTER_TRANSMIT();
    }
    END_ATOMAR;
}

//--------------------------------------------------------------------------
void ibus_recieveCallback(uint8_t c, uint16_t error)
{
    /*static uint8_t on = 0;
    if (on)
        PORTC &= ~(1 << 2);
    else
        PORTC |= (1 << 2);
    on = !on;*/

    // check if there was an error, then reset state and wait until bus get free
    if (error != 0)
    {
        /*static uint8_t onb = 0;
        if (onb)
            PORTC &= ~(1 << 5);
        else
            PORTC |= (1 << 5);
        onb = !onb;*/

        g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
        IBUS_TIMEOUT_RECEIVE_ERROR();
        return;
    }

    BEGIN_ATOMAR;
    {
        // go to recieve state and start timer which check for timeout
        g_ibus_State = IBUS_STATE_RECIEVE;

        // put recieved byte into buffer
        g_ibus_RxBuffer[g_ibus_RxPos] = c; inc_posptr_rx(g_ibus_RxPos);

        // notice length of the message
        if (g_ibus_RxPos == 2)
            g_ibus_RxLen = c;

        // if there were enough bytes recieved, then compare checksum
        else if (g_ibus_RxPos > 2 && (g_ibus_RxLen + 2 - g_ibus_RxPos) == 0)
        {
            IBUS_TIMER_DISABLE_INTERRUPT();

            g_ibus_RxPos = 0;
            g_ibus_RxLen = 0;

            uint8_t chk = ibus_calcChecksum(&g_ibus_RxBuffer[0]);
            if (chk != c) return;
        }
    }
    END_ATOMAR;

    // set timer for receive timeout
    IBUS_TIMEOUT_RECEIVE();

    if (g_ibus_MsgCallback)
        g_ibus_MsgCallback(g_ibus_RxBuffer[0], g_ibus_RxBuffer[2], &g_ibus_RxBuffer[3], g_ibus_RxBuffer[1]-2);
}

//--------------------------------------------------------------------------
void ibus_uartReceiveCallback(void)
{
    if (g_ibus_State == IBUS_STATE_RECIEVE || g_ibus_State == IBUS_STATE_IDLE)
    {
        unsigned int data = uart_getc();
        ibus_recieveCallback(data & 0xFF, data & 0xFF00);
    }
}

//--------------------------------------------------------------------------
void ibus_sendMessage(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msgLength, uint8_t numberOfTries)
{
    uint16_t oldWritePos = g_ibus_TxWritePos;
    uint8_t i;

    // check if we have enough space in the buffer, if not then don't take this msg
    int free = IBUS_MSG_TX_BUFFER_SIZE - (IBUS_MSG_TX_BUFFER_SIZE_MASK + g_ibus_TxWritePos - g_ibus_TxReadPos) % IBUS_MSG_TX_BUFFER_SIZE_MASK;
    if (free < msgLength+4) return;

    BEGIN_ATOMAR;
    {
        // put message into queue
        g_ibus_TxBuffer[g_ibus_TxWritePos] = src; inc_posptr_tx(g_ibus_TxWritePos);
        g_ibus_TxBuffer[g_ibus_TxWritePos] = msgLength + 2; inc_posptr_tx(g_ibus_TxWritePos);
        g_ibus_TxBuffer[g_ibus_TxWritePos] = dst; inc_posptr_tx(g_ibus_TxWritePos);
        for (i=0; i < msgLength; i++)
        {
            g_ibus_TxBuffer[g_ibus_TxWritePos] = msg[i]; inc_posptr_tx(g_ibus_TxWritePos);
        }
        g_ibus_TxBuffer[g_ibus_TxWritePos] = ibus_calcChecksum(&g_ibus_TxBuffer[oldWritePos]); inc_posptr_tx(g_ibus_TxWritePos);
        g_ibus_TxBuffer[g_ibus_TxWritePos] = numberOfTries; inc_posptr_tx(g_ibus_TxWritePos);
    }
    END_ATOMAR;
}

//--------------------------------------------------------------------------
void ibus_tick()
{
    // if we are currently waiting for free bus, then just do nothing
    if (g_ibus_TxReadPos == g_ibus_TxWritePos || g_ibus_State == IBUS_STATE_WAIT_FREE_BUS) return;

    // if there is currently a collision on the bus, then start timer
    // to wait for collision free bus again
    if (g_ibus_State == IBUS_STATE_IDLE && IBUS_SENSTA_VALUE())
    {
        g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
        IBUS_TIMEOUT_WAIT_FREE_BUS();
        return;
    }

    if (g_ibus_State != IBUS_STATE_IDLE) return;

    uint8_t len = g_ibus_TxBuffer[(g_ibus_TxReadPos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK];
    int8_t tried = g_ibus_TxBuffer[(g_ibus_TxReadPos + 2 + (posptr_t)len) & IBUS_MSG_TX_BUFFER_SIZE_MASK];

    // if we tried more than requested, then remove this message from the buffer and stop transmission
    if (tried < 1)
    {
        g_ibus_TxReadPos = (g_ibus_TxReadPos + 3 + (posptr_t)len) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
        g_ibus_TxReadPos_old = g_ibus_TxReadPos;
        return;
    }

    uart_flush();
    uart_clearTransmissionBuffer();
    
    g_ibus_State = IBUS_STATE_TRANSMIT;
    g_ibus_TxReadPos_old = g_ibus_TxReadPos;

    // put message into uart queue, do this atomar, so that full message is copied
    // before anything else happens
    BEGIN_ATOMAR;
    {
        // send full message
        posptr_t i;
        for (i=0; i < (posptr_t)len + 2; i++)
        {
            uart_putc(g_ibus_TxBuffer[g_ibus_TxReadPos], 1);
            inc_posptr_tx(g_ibus_TxReadPos);
        }

        // now decrement the number of tries how often this messages was resent
        g_ibus_TxBuffer[g_ibus_TxReadPos]--; inc_posptr_tx(g_ibus_TxReadPos);
    }
    END_ATOMAR;

    PORTD |= (1 << 6);

    // start transmission
    uart_setTxRx(1,0);
    uart_setTransmitDoneCallback(ibus_transmitCallback);
    uart_setReceiveCallback(NULL);
    uart_startTransmission();

    // react on bus collisions
    IBUS_SENSTA_ENABLE_INTERRUPT();
}

//--------------------------------------------------------------------------
void ibus_init()
{
    // initialize uart interface used for IBus communication
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
    uart_setFormat(8,1,1);
    uart_setReceiveCallback(ibus_uartReceiveCallback);
    uart_setTransmitDoneCallback(NULL);

    // setup pull-up on RX-pin
    IBUS_UART_RX_PULLUP_ENABLE();
    
    // per default we do not use any uart interface
    g_ibus_TxReadPos = 0;
    g_ibus_TxReadPos_old = 0;
    g_ibus_TxWritePos = 0;
    g_ibus_RxPos = 0;
    g_ibus_RxLen = 0;
    g_ibus_State = IBUS_STATE_IDLE;

    IBUS_SENSTA_SETUP();
    IBUS_TIMER_SETUP();
}

//--------------------------------------------------------------------------
ISR(IBUS_SENSTA_INT_VECT)
{
   /*     // debug
    static uint8_t on = 0;
    if (on)
        PORTD &= ~(1 << 5);
    else
        PORTD |= (1 << 5);
    on = !on;*/

    IBUS_SENSTA_DISABLE_INTERRUPT();
    
    BEGIN_ATOMAR;
    {
        // clear buffers and set everything back, so that msg is retransmitted
        uart_clearTransmissionBuffer();
        uart_flush();
        g_ibus_TxReadPos = g_ibus_TxReadPos_old;
        g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
        
        // we disable recieve and transmit modes
        uart_setTxRx(0,0);
        uart_setTransmitDoneCallback(NULL);
        uart_setReceiveCallback(NULL);
    }
    END_ATOMAR;

    // start timer to wait until we change the state, so that no transmission
    // happens during this time
    IBUS_TIMEOUT_COLLISION();
}

//--------------------------------------------------------------------------
// This timer is a wait timer, so the execution of IBus protocl is stopped
// while timer is running
//--------------------------------------------------------------------------
ISR(IBUS_TIMER_INTERRUPT)
{
    // disable timer interrupts
    IBUS_TIMER_DISABLE_INTERRUPT();

    // ok go to idle state
    g_ibus_State = IBUS_STATE_IDLE;
    g_ibus_RxPos = 0;
    g_ibus_RxLen = 0;

    // enable reciever, disable transmitter
    uart_flush();
    uart_setTxRx(0,1);
    uart_setTransmitDoneCallback(NULL);
    uart_setReceiveCallback(ibus_uartReceiveCallback);
}
