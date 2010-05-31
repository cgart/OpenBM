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

// singleton
IBus IBus::mSingleton;
uint8_t IBus::mTxBuffer[IBUS_MSG_TX_BUFFER_SIZE];
uint8_t IBus::mRxBuffer[IBUS_MSG_RX_BUFFER_SIZE];

//--------------------------------------------------------------------------
IBus* IBus::initialize()
{
    // intialize pointer
    IBus* ptr = &IBus::mSingleton;

    // initialize uart interface used for IBus communication
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
    uart_setFormat(8,1,1);
    uart_setReceiveCallback(IBus::uartReceiveCallback);
    
    // per default we do not use any uart interface
    ptr->mTxReadPos = 0;
    ptr->mTxReadPos_old = 0;
    ptr->mTxWritePos = 0;
    ptr->mRxPos = 0;
    ptr->mRxLen = 0;
    ptr->mState = IBUS_STATE_IDLE;
    
    IBUS_SENSTA_SETUP();
    IBUS_TIMER_SETUP();

    // return interface pointer
    return ptr;
}

//--------------------------------------------------------------------------
void IBus::uartTransmittedCallback(void)
{
    IBus::mSingleton.transmitCallback();
}

//--------------------------------------------------------------------------
void IBus::uartReceiveCallback()
{
    unsigned int data = uart_getc();
    IBus::mSingleton.recieveCallback(data & 0xFF, data & 0xFF00);
}

//--------------------------------------------------------------------------
void IBus::transmitCallback()
{
    BEGIN_ATOMAR;
    {
        // debug
        static bool on = false;
        if (on)
            PORTD &= ~(1 << 6);
        else
            PORTD |= (1 << 6);
        on = !on;

        // disable collision interrupt and
        IBUS_SENSTA_DISABLE_INTERRUPT();
        uart_setTransmitDoneCallback(NULL);
        
        // start to wait for free bus, this will introduce small delay between msgs
        mState = IBUS_STATE_WAIT_FREE_BUS;
        IBUS_TIMER_30MS();
    }
    END_ATOMAR;
}

//--------------------------------------------------------------------------
void IBus::recieveCallback(uint8_t c, uint16_t error)
{
            static bool on = false;
        if (on)
            PORTC &= ~(1 << 5);
        else
            PORTC |= (1 << 5);
        on = !on;


    // check if there was a frame error, then reset state and wait until bus get free
    if (error == UART_FRAME_ERROR)
    {
        static bool on = false;
        if (on)
            PORTC &= ~(1 << 2);
        else
            PORTC |= (1 << 2);
        on = !on;

        mState = IBUS_STATE_WAIT_FREE_BUS;
        IBUS_TIMER_30MS();
        return;
    }else if (error != 0)
        return;

    BEGIN_ATOMAR;
    {
        // go to recieve state and start timer which check for timeout
        mState = IBUS_STATE_RECIEVE;

        // put recieved byte into buffer
        mRxBuffer[mRxPos] = c;
        mRxPos = (mRxPos + 1) & IBUS_MSG_RX_BUFFER_SIZE_MASK;

        // notice length of the message
        if (mRxPos == 2)
            mRxLen = c;

        // if there were enough bytes recieved, then compare checksum
        else if (mRxPos > 2 && (mRxLen + 2 - mRxPos) == 0)
        {
            IBUS_TIMER_DISABLE_INTERRUPT();

            mRxPos = 0;
            mRxLen = 0;

            uint8_t chk = calcChecksum(&mRxBuffer[0]);
            if (chk != c) return;
        }
    }
    END_ATOMAR;

    if (mMsgCallback)
        mMsgCallback(mRxBuffer[0], mRxBuffer[2], &mRxBuffer[3], mRxBuffer[1]-2);

    // set timer for receive timeout
    IBUS_TIMER_75MS();
}

//--------------------------------------------------------------------------
void IBus::sendMessage(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msgLength, uint8_t numberOfTries)
{
    uint16_t oldWritePos = mTxWritePos;

    // check if we have enough space in the buffer, if not then don't take this msg
    int free = IBUS_MSG_TX_BUFFER_SIZE - (IBUS_MSG_TX_BUFFER_SIZE_MASK + mTxWritePos - mTxReadPos) % IBUS_MSG_TX_BUFFER_SIZE_MASK;
    if (free < msgLength+4) return;

    BEGIN_ATOMAR;
    {
        // put message into queue
        mTxBuffer[mTxWritePos] = src;
        mTxWritePos = (mTxWritePos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
        mTxBuffer[mTxWritePos] = msgLength + 2;
        mTxWritePos = (mTxWritePos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
        mTxBuffer[mTxWritePos] = dst;
        mTxWritePos = (mTxWritePos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
        for (uint8_t i=0; i < msgLength; i++)
        {
            mTxBuffer[mTxWritePos] = msg[i];
            mTxWritePos = (mTxWritePos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
        }
        mTxBuffer[mTxWritePos] = calcChecksum(&mTxBuffer[oldWritePos]);
        mTxWritePos = (mTxWritePos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
        mTxBuffer[mTxWritePos] = numberOfTries;
        mTxWritePos = (mTxWritePos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
    }
    END_ATOMAR;
}

//--------------------------------------------------------------------------
void IBus::tick()
{
    if (mState == IBUS_STATE_IDLE)
        PORTD |= (1 << 7);
    else
        PORTD &= ~(1 << 7);

    // if we are currently waiting for free bus, then just do nothing
    if (mTxReadPos == mTxWritePos || mState == IBUS_STATE_WAIT_FREE_BUS) return;

    // if there is currently a collision on the bus, then start timer
    // to wait for collision free bus again
    if (mState == IBUS_STATE_IDLE && IBUS_SENSTA_VALUE())
    {
        mState = IBUS_STATE_WAIT_FREE_BUS;
        IBUS_TIMER_75MS();
        return;
    }

    // at this moment, we only accept bus in idle state, otherwise return
    if (mState != IBUS_STATE_IDLE) return;

    // check how often we already tried to send this message
    // if we tried more than requested, then remove this message from the
    // buffer and stop transmission
    {
        uint8_t len = mTxBuffer[(mTxReadPos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK];
        char tries = mTxBuffer[(mTxReadPos + 2 + (uint16_t)len) & IBUS_MSG_TX_BUFFER_SIZE_MASK];
        if (tries < 1)
        {
            mTxReadPos = (mTxReadPos + 3 + (uint16_t)len) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
            mTxReadPos_old = mTxReadPos;
            return;
        }
    }

    // go to transmit state and set rx pos to starting of transmited pos,
    // so that we can check transmitted bytes
    uart_clearTransmissionBuffer();
    mState = IBUS_STATE_TRANSMIT;
    mTxReadPos_old = mTxReadPos;

    // put one message into fifo of uart interface
    uint8_t src = mTxBuffer[mTxReadPos];
    uart_putc(src, 1);
    mTxReadPos = (mTxReadPos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
    uint8_t len = mTxBuffer[mTxReadPos];
    uart_putc(len, 1);
    mTxReadPos = (mTxReadPos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
    for (uint8_t i=0; i < len; i++)
    {
        uart_putc(mTxBuffer[mTxReadPos], 1);
        mTxReadPos = (mTxReadPos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
    }

    // now decrement the number of tries how often this messages was resent
    mTxBuffer[mTxReadPos]--;
    mTxReadPos = (mTxReadPos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
    mRxPos = mTxReadPos_old;

    // react on bus collisions
    IBUS_SENSTA_ENABLE_INTERRUPT();

    // start transmission
    uart_setTxRx(1,0);
    uart_flush();
    uart_setTransmitDoneCallback(IBus::uartTransmittedCallback);
    uart_startTransmission();
}

//--------------------------------------------------------------------------
uint8_t IBus::calcChecksum(uint8_t* pBuffer)
{
  uint8_t i;
  uint8_t checksum;
  uint8_t len;

  checksum = 0;
  len = pBuffer[1] + 1;

  if(pBuffer == NULL)
    return 0;

  for(i = 0; i < len; i++)
  {
      i &= IBUS_MSG_TX_BUFFER_SIZE_MASK;
      checksum ^= pBuffer[i];
  }

  return checksum;
}

//--------------------------------------------------------------------------
//void IBus::onCollision()
ISR(IBUS_SENSTA_INT_VECT)
{
        // debug
    static bool on = false;
    if (on)
        PORTD &= ~(1 << 5);
    else
        PORTD |= (1 << 5);
    on = !on;

    // global instance
    static IBus* ptr = &IBus::mSingleton;

    IBUS_SENSTA_DISABLE_INTERRUPT();
    
    BEGIN_ATOMAR;
    {
        // clear buffers and set everything back, so that msg is retransmitted
        uart_clearTransmissionBuffer();
        uart_flush();
        ptr->mTxReadPos = ptr->mTxReadPos_old;
        ptr->mState = IBUS_STATE_WAIT_FREE_BUS;
        uart_setTransmitDoneCallback(NULL);
        
        // we disable recieve and transmit modes
        uart_setTxRx(0,0);
    }
    END_ATOMAR;

    // start timer to wait until we change the state, so that no transmission
    // happens during this time
    IBUS_TIMER_100MS();
}

//--------------------------------------------------------------------------
// This timer is a wait timer, so the execution of IBus protocl is stopped
// while timer is running
//--------------------------------------------------------------------------
ISR(IBUS_TIMER_INTERRUPT)
{
    static IBus* ptr = &IBus::mSingleton;

    // disable timer interrupts
    IBUS_TIMER_DISABLE_INTERRUPT();

    // ok go to idle state
    ptr->mState = IBUS_STATE_IDLE;
    ptr->mRxPos = 0;
    ptr->mRxLen = 0;

    // enable reciever, disable transmitter
    uart_setTxRx(0,1);
}
