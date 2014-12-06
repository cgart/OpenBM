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

#include "ibus.h"
#include "uart.h"
#include "leds.h"
#include "include/leds.h"
#include "include/ibus.h"

#define IBUS_USE_SHORT_BUFFER

#ifdef IBUS_USE_SHORT_BUFFER
    #define IBUS_MSG_TX_BUFFER_SIZE       64
    #define IBUS_MSG_RX_BUFFER_SIZE       128

    typedef uint8_t posptr_t;
#else
    #define IBUS_MSG_TX_BUFFER_SIZE       256
    #define IBUS_MSG_RX_BUFFER_SIZE       256

    typedef uint16_t posptr_t;
#endif

#define IBUS_MSG_TX_BUFFER_SIZE_MASK  (IBUS_MSG_TX_BUFFER_SIZE - 1)
#define IBUS_MSG_RX_BUFFER_SIZE_MASK  (IBUS_MSG_RX_BUFFER_SIZE - 1)

#define inc_posptr(ptr, mask) ((ptr+1) & mask)
#define inc_posptr_rx(ptr) ((ptr+1) & IBUS_MSG_RX_BUFFER_SIZE_MASK)
#define inc_posptr_tx(ptr) ((ptr+1) & IBUS_MSG_TX_BUFFER_SIZE_MASK)


uint8_t g_ibus_TxBuffer[IBUS_MSG_TX_BUFFER_SIZE];
uint8_t g_ibus_RxBuffer[IBUS_MSG_RX_BUFFER_SIZE];

void(*g_ibus_MsgCallback)(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen);

volatile uint8_t  g_ibus_State ;
volatile posptr_t g_ibus_RxPos ;
volatile posptr_t g_ibus_RxLen ;
posptr_t g_ibus_TxReadPos ;
posptr_t g_ibus_TxWritePos ;

//--------------------------------------------------------------------------
void ibus_setTimeOut(uint8_t delay)
{
    TCNT0 = 255 - delay;
    TIMSK0 |= (1 << TOIE0);
    TIFR0 |= (1 << TOV0);
}

//--------------------------------------------------------------------------
// Transmit message over the bus. Message is readed from the given
// buffer position. While transmitting we check if bus is free and if
// same bit was recieved as transmitted. This ensures correct handling
// This method is time critical and interrupts will be disabled.
//--------------------------------------------------------------------------
posptr_t ibus_transmit_msg(posptr_t from, posptr_t len)
{
    posptr_t oldFrom = from;


#ifdef RXTX_COMPARE
    if (g_ibus_State != IBUS_STATE_TRANSMITTING) return from;
    IBUS_RX_DIS_INT();
#else
    if (IBUS_SENSTA_VALUE()) return from;
    // we currently disable interrupt of TH3122, we will ask it manually here
    IBUS_SENSTA_DIS_INT();
#endif

    while(len > 0)
    {
        // load byte to transmit
        uint8_t byte = g_ibus_TxBuffer[from];
        uint8_t sent = byte;
        uint8_t par  = 0;

        // transmit start bit and check if collision happens
        bit_clear(IBUS_TX_PORT, IBUS_TX_PIN);
        #ifdef RXTX_COMPARE
            IBUS_BAUD_HDELAY();
            if (IBUS_RX_VALUE() != 0) goto collision;
            IBUS_BAUD_HDELAY();
        #else
            IBUS_BAUD_DELAY();
            if (IBUS_SENSTA_VALUE()) goto collision;
        #endif

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

            #ifdef RXTX_COMPARE
                IBUS_BAUD_HDELAY();
                if (IBUS_RX_VALUE() != (byte & 1)) goto collision;
                IBUS_BAUD_HDELAY();
            #else
                IBUS_BAUD_DELAY();
                if (IBUS_SENSTA_VALUE()) goto collision;
            #endif
        }

        // send parity bit
        if (par & 1)
            bit_set(IBUS_TX_PORT, IBUS_TX_PIN);
        else
            bit_clear(IBUS_TX_PORT, IBUS_TX_PIN);

        #ifdef RXTX_COMPARE
            IBUS_BAUD_HDELAY();
            if (IBUS_RX_VALUE() != (par & 1)) goto collision;
            IBUS_BAUD_HDELAY();
        #else
            IBUS_BAUD_DELAY();
            if (IBUS_SENSTA_VALUE()) goto collision;
        #endif

        // send stop bit
        bit_set(IBUS_TX_PORT, IBUS_TX_PIN);
        #ifdef RXTX_COMPARE
            IBUS_BAUD_HDELAY();
            if (IBUS_RX_VALUE() != 1) goto collision;
            IBUS_BAUD_HDELAY();
        #else
            IBUS_BAUD_DELAY();
            if (IBUS_SENSTA_VALUE()) goto collision;
        #endif

        // update current pointers
        from = inc_posptr_tx(from);
        len--;

        // we should have now received the same byte through uart rx
        // check if there was no frame error and if the received byte is the right one
        if (!uart_available()) goto collision;
        if (uart_lastc() != sent) goto collision;
        if (uart_last_error()) goto collision;
    }

    // if everything went fine, then we will return new from position
    oldFrom = from;


// return on failure
collision:

    IBUS_TX_SETUP();
    #ifdef RXTX_COMPARE
        IBUS_RX_ENA_INT();
    #else
        IBUS_SENSTA_ENA_INT();
    #endif
    //END_ATOMAR;
    return oldFrom;
}

//--------------------------------------------------------------------------
void ibus_setMessageCallback(void(*cb)(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen))
{
    g_ibus_MsgCallback = cb;
}

//--------------------------------------------------------------------------
uint8_t ibus_calcChecksum(const uint8_t* pBuffer, posptr_t from, uint8_t mask)
{
    uint8_t checksum = pBuffer[from]; from = inc_posptr(from, mask);
    uint8_t len = pBuffer[from];
    while(len > 0)
    {
        checksum ^= pBuffer[from];
        len --;
        from = inc_posptr(from, mask);
    }

    return checksum;
}

//--------------------------------------------------------------------------
void ibus_recieveCallback(uint8_t c)
{
    // go to recieve state and start timer which check for timeout
    g_ibus_State = IBUS_STATE_RECEIVING;
    IBUS_TIMEOUT_RECEIVE();

    uint8_t msgReceived = 0;
    
    {
        // put recieved byte into buffer
        g_ibus_RxBuffer[g_ibus_RxPos] = c;

        // notice length of the message
        if (g_ibus_RxPos == 1)
        {
            g_ibus_RxLen = c + 2;
        }

        g_ibus_RxPos = inc_posptr_rx(g_ibus_RxPos);

        // if there were enough bytes recieved, then compare checksum
        if (g_ibus_RxPos >= g_ibus_RxLen)
        {
            IBUS_TIMER_DISABLE_INTERRUPT();

            g_ibus_RxPos = 0;
            g_ibus_RxLen = 2;

            uint8_t chk = ibus_calcChecksum(g_ibus_RxBuffer, 0, IBUS_MSG_RX_BUFFER_SIZE_MASK);
            msgReceived = (chk == c);

            g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
            IBUS_TIMEOUT_AFTER_RECEIVE();
        }
    }

    if (msgReceived && g_ibus_MsgCallback)
        g_ibus_MsgCallback(g_ibus_RxBuffer[0], g_ibus_RxBuffer[2], &g_ibus_RxBuffer[3], g_ibus_RxBuffer[1]-2);
}

//--------------------------------------------------------------------------
void ibus_sendMessage(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msgLength, uint8_t numberOfTries)
{
    // put number of retries
    g_ibus_TxBuffer[g_ibus_TxWritePos] = numberOfTries; g_ibus_TxWritePos = inc_posptr_tx(g_ibus_TxWritePos);

    posptr_t oldWritePos = g_ibus_TxWritePos;

    // put message into queue
    g_ibus_TxBuffer[g_ibus_TxWritePos] = src; g_ibus_TxWritePos = inc_posptr_tx(g_ibus_TxWritePos);
    g_ibus_TxBuffer[g_ibus_TxWritePos] = msgLength + 2; g_ibus_TxWritePos = inc_posptr_tx(g_ibus_TxWritePos);
    g_ibus_TxBuffer[g_ibus_TxWritePos] = dst; g_ibus_TxWritePos = inc_posptr_tx(g_ibus_TxWritePos);
    for (uint8_t i=0; i < msgLength; i++)
    {
        g_ibus_TxBuffer[g_ibus_TxWritePos] = msg[i];
        g_ibus_TxWritePos = inc_posptr_tx(g_ibus_TxWritePos);
    }
    g_ibus_TxBuffer[g_ibus_TxWritePos] = ibus_calcChecksum(g_ibus_TxBuffer, oldWritePos, IBUS_MSG_TX_BUFFER_SIZE_MASK);
    g_ibus_TxWritePos = inc_posptr_tx(g_ibus_TxWritePos);
}

//--------------------------------------------------------------------------
uint8_t ibus_isQueueFree()
{
    return (g_ibus_TxReadPos == g_ibus_TxWritePos);
}

//--------------------------------------------------------------------------
uint8_t ibus_readyToTransmit()
{
    return (g_ibus_TxReadPos != g_ibus_TxWritePos && g_ibus_State == IBUS_STATE_IDLE);
}

//--------------------------------------------------------------------------
void ibus_tick()
{
    // if we have data in the receive buffer, then handle it
    while(uart_available())
    {
        uint8_t data = uart_getc();

        if (uart_last_error())
        {
            uart_clear_error();
            g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
            IBUS_TIMEOUT_RECEIVE_ERROR();
            break;
        }else
            ibus_recieveCallback(data);
    }

    // if no data in the buffer or we are not idle, then do nothing
    if (!ibus_readyToTransmit()) return;
    
    // ok we are idle and we have data in the buffer, then first, wait for free buffer
    #ifndef RXTX_COMPARE
    if (IBUS_SENSTA_VALUE())
    {
        g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
        IBUS_TIMEOUT_WAIT_FREE_BUS();
        return;
    }
    #endif

    g_ibus_State = IBUS_STATE_TRANSMITTING;
    uart_clear_error(); // start fresh :)
    
    // ok bus is free, we can submit a message
    posptr_t tryCounterPos = g_ibus_TxReadPos;
    int8_t numberOfTries = g_ibus_TxBuffer[g_ibus_TxReadPos]; g_ibus_TxReadPos = inc_posptr_tx(g_ibus_TxReadPos);
    posptr_t len = (posptr_t)g_ibus_TxBuffer[(g_ibus_TxReadPos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK];

    // transmit message
    posptr_t oldTxPos = g_ibus_TxReadPos;
    posptr_t newTxPos = ibus_transmit_msg(g_ibus_TxReadPos,  len + 2);
    
    // from here we are free to clear all errors
    uart_clear_error();

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

            return;
        }
        newTxPos = (posptr_t)(oldTxPos + len + 2) & IBUS_MSG_TX_BUFFER_SIZE_MASK;

    // if there was no collision, then we have received the same message as sent, so just flush it
    }else
        uart_flush(0);

    // ok message was either submitted successfully or message trials
    // counter is elapsed, we flush tx buffer and continue
    g_ibus_TxReadPos = newTxPos;

    g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
    IBUS_TIMEOUT_AFTER_TRANSMIT();
}

//--------------------------------------------------------------------------
void ibus_init()
{
    // initialize uart interface used for IBus communication, uart default 9600 8E1
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
    uart_setRx(1);

    g_ibus_MsgCallback = NULL;

    // per default we do not use any uart interface
    g_ibus_TxReadPos = 0;
    g_ibus_TxWritePos = 0;
    g_ibus_RxPos = 0;
    g_ibus_RxLen = 2;
    g_ibus_State = IBUS_STATE_IDLE;

    IBUS_TIMER_SETUP();
    IBUS_TX_SETUP();

    // currently no timer is running
    IBUS_TIMER_DISABLE_INTERRUPT();

    #ifdef RXTX_COMPARE
        PCICR |= (1 << PCIE3);     // pin change interrupt on RxD0 pin
        PCMSK3 |= (1 << PCINT24);
        
    #else
        IBUS_SENSTA_SETUP();

        // interrupt on falling edge on SEN/STA pin
        EIMSK &= ~(1 << INT0);
        EICRA &= ~(1 << ISC00);
        EICRA |=  (1 << ISC01);
        EIFR  |=  (1 << INTF0);
    #endif

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
}

//--------------------------------------------------------------------------
// When SEN/STA down, then trigger this interrupt
//--------------------------------------------------------------------------
#ifdef RXTX_COMPARE
ISR(PCINT3_vect)
#else
ISR(INT0_vect)
#endif
{
    if (g_ibus_State == IBUS_STATE_TRANSMITTING) return;
    
    g_ibus_State = IBUS_STATE_WAIT_FREE_BUS;
    IBUS_TIMEOUT_WAIT_FREE_BUS();
}
