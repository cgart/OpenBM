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

// singleton
IBus IBus::mSingleton;
uint8_t IBus::mTxBuffer[IBUS_MSG_TX_BUFFER_SIZE];
uint8_t IBus::mRxBuffer[IBUS_MSG_RX_BUFFER_SIZE];

//--------------------------------------------------------------------------
IBus* IBus::initialize()
{
    // intialize pointer
    IBus* ptr = &IBus::mSingleton;

    // per default we do not use any uart interface
    ptr->mBusUart = NULL;
    ptr->mTxReadPos = 0;
    ptr->mTxReadPos_old = 0;
    ptr->mTxWritePos = 0;
    ptr->mRxPos = 0;
    ptr->mRxLen = 0;
    ptr->mState = IDLE;
    
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
void IBus::uartReceiveCallback(uint8_t c)
{
    IBus::mSingleton.recieveCallback(c);

    // debug
    static bool on = false;
    if (on)
        PORTD &= ~(1 << 5);
    else
        PORTD |= (1 << 5);
    on = !on;
}

//--------------------------------------------------------------------------
void IBus::transmitCallback()
{
    // debug
    static bool on = false;
    if (on)
        PORTD &= ~(1 << 6);
    else
        PORTD |= (1 << 6);
    on = !on;

    // disable interrupts
    IBUS_SENSTA_DISABLE_INTERRUPT();

    // ok transmission went fine, so go to idle state
    mBusUart->setTransmitDoneCallback(NULL);
    mBusUart->setRxEnable(1);
    mBusUart->setTxEnable(0);
    mState = IDLE;
}

//--------------------------------------------------------------------------
void IBus::recieveCallback(uint8_t c)
{
    // go to recieve state and start timer which check for timeout
    mState = RECIEVE;
    IBUS_TIMER_100MS();
    
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

        if (mMsgCallback)
            mMsgCallback(mRxBuffer[0], mRxBuffer[2], &mRxBuffer[3], mRxBuffer[1]-2);
    }
}

//--------------------------------------------------------------------------
void IBus::sendMessage(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msgLength)//, uint8_t numberOfTries)
{
    uint16_t oldWritePos = mTxWritePos;
    
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
}

//--------------------------------------------------------------------------
void IBus::tick()
{
    // debug
    static bool on = false;
    if (on)
        PORTC &= ~(1 << 2);
    else
        PORTC |= (1 << 2);
    on = !on;

    if (IBUS_SENSTA_VALUE() || mState != IDLE || mTxReadPos == mTxWritePos) return;
    
    mState = TRANSMIT;    
    mTxReadPos_old = mTxReadPos;

    // prepare hardware to transmit data
    // enable interrupt to react on bus collisions
    IBUS_SENSTA_ENABLE_INTERRUPT();
    mBusUart->setRxEnable(0);
    mBusUart->setTxEnable(1);
    mBusUart->setTransmitDoneCallback(IBus::uartTransmittedCallback);
    
    // put one message into fifo of uart interface
    uint8_t src = mTxBuffer[mTxReadPos]; 
    mBusUart->sendByte(src);
    mTxReadPos = (mTxReadPos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
    uint8_t len = mTxBuffer[mTxReadPos];
    mBusUart->sendByte(len);
    mTxReadPos = (mTxReadPos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
    for (uint8_t i=0; i < len; i++)
    {
        if (mTxReadPos == mTxReadPos_old) break;  // this happens only if end reached or there was a transmission failure
        mBusUart->sendByte(mTxBuffer[mTxReadPos]);
        mTxReadPos = (mTxReadPos + 1) & IBUS_MSG_TX_BUFFER_SIZE_MASK;
    }
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
    checksum ^= pBuffer[i];
  }

  return checksum;
}

//--------------------------------------------------------------------------
ISR(IBUS_SENSTA_INT_VECT)
{
    static IBus* ptr = &IBus::mSingleton;

    BEGIN_ATOMAR;
    {
        // almost the same as when message was transmitted successfully
        IBUS_SENSTA_DISABLE_INTERRUPT();

        // set everything back, so that message get retransmitted next
        ptr->mTxReadPos = ptr->mTxReadPos_old;
        ptr->mBusUart->clearTransmitBuffer();
        ptr->mBusUart->setTransmitDoneCallback(NULL);

        // we disable recieve and transmit modes
        ptr->mBusUart->setRxEnable(0);
        ptr->mBusUart->setTxEnable(0);

        // start timer to wait until we change the state, so that no transmission
        // happens during this time
        IBUS_TIMER_50MS();
    }
    END_ATOMAR;
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

    // ok go to idle state and allow recieve again
    ptr->mState = IBus::IDLE;
    ptr->mBusUart->setRxEnable(1);
    ptr->mBusUart->setTxEnable(0);
    ptr->mRxPos = 0;
    ptr->mRxLen = 0;
}
