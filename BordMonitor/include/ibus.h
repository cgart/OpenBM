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
#ifndef IBUS_CLASS_H
#define IBUS_CLASS_H

//--------------------------------------------------------------------------
// Include basic headers
//--------------------------------------------------------------------------
#include "base.h"
#include <string.h>

//*** Device Codes ***
#define IBUS_DEV_GM      0X00    // Body Module
#define IBUS_DEV_CDC     0x18    // CD Changer
#define IBUS_DEV_FUH     0x28    // Radio controlled clock
#define IBUS_DEV_CCM     0x30    // Check control module
#define IBUS_DEV_GT      0x3B    // Graphics driver (in navigation system)
#define IBUS_DEV_DIA     0x3F    // Diagnostic
#define IBUS_DEV_FBZV    0x40    // Remote control central locking
#define IBUS_DEV_GTF     0x43    // Graphics driver for rear screen (in navigation system)
#define IBUS_DEV_EWS     0x44    // Immobiliser
#define IBUS_DEV_CID     0x46    // Central information display (flip-up LCD screen)
#define IBUS_DEV_MFL     0x50    // Multi function steering wheel
#define IBUS_DEV_MM      0x51    // Mirror memory
#define IBUS_DEV_IHK     0x5B    // Integrated heating and air conditioning
#define IBUS_DEV_PDC     0x60    // Park distance control
#define IBUS_DEV_ONL     0x67    // unknown
#define IBUS_DEV_RAD     0x68    // Radio
#define IBUS_DEV_DSP     0x6A    // Digital signal processing audio amplifier
#define IBUS_DEV_SM1     0x72    // Seat memory
#define IBUS_DEV_CDCD    0x76    // CD changer, DIN size.
#define IBUS_DEV_NAVE    0x7F    // Navigation (Europe)
#define IBUS_DEV_IKE     0x80    // Instrument cluster electronics
#define IBUS_DEV_MM1     0x9B    // Mirror memory
#define IBUS_DEV_MM2     0x9C    // Mirror memory
#define IBUS_DEV_FMID    0xA0    // Rear multi-info-display
#define IBUS_DEV_ABM     0xA4    // Air bag module
#define IBUS_DEV_KAM     0xA8    // unknown
#define IBUS_DEV_ASP     0xAC    // unknown
#define IBUS_DEV_SES     0xB0    // Speed recognition system
#define IBUS_DEV_NAVJ    0xBB    // Navigation (Japan)
#define IBUS_DEV_GLO     0xBF    // Global, broadcast address
#define IBUS_DEV_MID     0xC0    // Multi-info display
#define IBUS_DEV_TEL     0xC8    // Telephone
#define IBUS_DEV_LCM     0xD0    // Light control module
#define IBUS_DEV_SM2     0xDA    // Seat memory
#define IBUS_DEV_GTHL    0xDA    // unknown
#define IBUS_DEV_IRIS    0xE0    // Integrated radio information system
#define IBUS_DEV_ANZV    0xE7    // Front display
#define IBUS_DEV_ISP     0xE8    // unknown
#define IBUS_DEV_TV      0xED    // Television
#define IBUS_DEV_BMBT    0xF0    // On-board monitor operating part
#define IBUS_DEV_CSU     0xF5    // unknown
#define IBUS_DEV_LOC     0xFF    // Local


//*** Message Types ***
#define IBUS_MSG_LAMP_STATE         0x5B    // Lamp state
#define IBUS_MSG_VEHICLE_CTRL       0x0C    // Vehicle Control (mostly used from diagnose)

//*** iBus Settings ***
#define IBUS_MSG_TX_BUFFER_SIZE       256
#define IBUS_MSG_TX_BUFFER_SIZE_MASK  (IBUS_MSG_TX_BUFFER_SIZE - 1)
#define IBUS_MSG_RX_BUFFER_SIZE       256
#define IBUS_MSG_RX_BUFFER_SIZE_MASK  (IBUS_MSG_RX_BUFFER_SIZE - 1)
#define IBUS_SENSTA_VALUE()           bit_is_set(PIND,2)
#define IBUS_SENSTA_SETUP()           { DDRD &= ~(1 << DDD2); PORTD |= (1 << 2);}
#define IBUS_SENSTA_INT_VECT          INT0_vect
#define IBUS_SENSTA_ENABLE_INTERRUPT()  { MCUCR |= (1 << ISC00) | (1 << ISC01); GICR |= (1 << INT0); }
#define IBUS_SENSTA_DISABLE_INTERRUPT()  { GICR &= ~(1 << INT0); }
#define IBUS_TIMER_SETUP() {TCCR1B = (1 << CS11) | (1 << CS10);}
#define IBUS_TIMER_100MS() { BEGIN_ATOMAR; TIMSK |= (1 << TOIE1); TCNT1 = 42496; END_ATOMAR;}
#define IBUS_TIMER_75MS()  { BEGIN_ATOMAR; TIMSK |= (1 << TOIE1); TCNT1 = 48256; END_ATOMAR;}
#define IBUS_TIMER_50MS()  { BEGIN_ATOMAR; TIMSK |= (1 << TOIE1); TCNT1 = 54016; END_ATOMAR;}
#define IBUS_TIMER_30MS()  { BEGIN_ATOMAR; TIMSK |= (1 << TOIE1); TCNT1 = 58624; END_ATOMAR;}
#define IBUS_TIMER_DISABLE_INTERRUPT() {TIMSK &= ~(1 << TOIE1);}
#define IBUS_TIMER_INTERRUPT TIMER1_OVF_vect
#define IBUS_TRANSMIT_TRIES 3


// default constants
#define IBUS_STATE_IDLE 0
#define IBUS_STATE_WAIT_FREE_BUS  (1 << 0)
#define IBUS_STATE_RECIEVE  (1 << 1)
#define IBUS_STATE_TRANSMIT  (1 << 2)

/**
 * Class to handle IBus messages. The I-Bus must be conencted
 * through a proper device to the uart connecter of the µC.
 * This device must handle the bus specific communication.
 * The logic is handled in this class blah blah blah :-)
 **/
class IBus
{
	public:


        typedef unsigned char State;
        State mState;
        
        /**
         * Initialize singleton pointer to the IBus
        **/
        static IBus* initialize();

        //! Singleton storing the interface
        static IBus mSingleton;
        
        /**
         * Callback function which will be used if a message on
         * the ibus is found.
         * @param src ID of the sender
         * @param dst ID of the receiver
         * @param msg String containing the full message
         * @param msglen Length of the message string
         **/
        typedef void(* MessageCallback)(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen);

        /**
         * Set callback function which will be called on a new message
         **/
        inline void setMessageCallback(MessageCallback f);

        /**
         * Callback to be used to send byte over uart interface
         **/
        typedef void(* UartSendByteCallback)(uint8_t byte);
        inline void setUartSendByteCallback(UartSendByteCallback f) { mSendByteCallback=f; }
        
        /**
         * Tick - perform updates, send messages from queue, submit recieved msgs
         **/
        void tick();
        
        /**
         * Calculate checksum of the message
         **/
        uint8_t calcChecksum(uint8_t* pBuffer);

        /**
         * Send message over the ibus. You can speify how much tries
         * there will be if message could not be send.
         **/
        void sendMessage(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msgLength, uint8_t numberOfTries);

    public:

	//! Callback function for new messages
        MessageCallback mMsgCallback;
        UartSendByteCallback mSendByteCallback;
        
        //! Function to use as callback for the uart interface
        static void uartReceiveCallback(void);
        static void uartTransmittedCallback(void);
        void recieveCallback(uint8_t c, uint16_t error);
        void transmitCallback();
        void startTransmission();
        
        //! transimt buffer where to hold messages to be transmitted
        static uint8_t mTxBuffer[];

        //! recieve buffer
        static uint8_t mRxBuffer[];
        
        //! current read position from the tx buffer
        uint16_t mTxReadPos_old;
        uint16_t mTxReadPos;
        uint16_t mTxWritePos;
        uint16_t mRxPos;
        uint8_t  mRxLen;
};


//--------------------------------------------------------------------------
void IBus::setMessageCallback(MessageCallback f)
{
    mMsgCallback = f;
}


#endif
