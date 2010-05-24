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
#include "UartLib.h"
#include "uart.h"
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


/**
 * Class to handle IBus messages. The I-Bus must be conencted
 * through a proper device to the uart connecter of the µC.
 * This device must handle the bus specific communication.
 * The logic is handled in this class blah blah blah :-)
 **/
class IBus
{
	public:

        /**
         * Initialize singleton pointer to the IBus
        **/
        static IBus* initialize();

        //! Singleton storing the interface
        static IBus mSingleton;

        //! Set uart interface to use for comunication
        inline void setUart(Uart* uart);

        /**
         * Callback function which will be used if a message on
         * the ibus is found.
         * @param src ID of the sender
         * @param dst ID of the receiver
         * @param len Length of the message string
         * @param msg String containing the full message
         * @return Type of the message
         **/
        typedef uint8_t(* MessageCallback)(uint8_t src, uint8_t dst, uint8_t len, uint8_t* msg);

        /**
         * Set callback function which will be called on a new message
         **/
        inline void setMessageCallback(MessageCallback f);

        /**
         * Tick - perform updates, send messages from queue, submit recieved msgs
         **/
        void tick();
        
    private:

        //! uart interface which is connected to the ibus
        Uart* mBusUart;

	//! Callback function for new messages
        MessageCallback mMsgCallback;

        //! Function to use as callback for the uart interface
        static void uartReceiveCallback(uint8_t c);

};

//--------------------------------------------------------------------------
void IBus::setUart(Uart* uart)
{
    mBusUart = uart;
    if (uart != NULL)
    {
        mBusUart->setBaud(UART_COMPUTE_BAUD(F_CPU, 9600));
        mBusUart->setFormat(8,1,Uart::EVEN_PARITY);
        mBusUart->setReceiveCallback(uartReceiveCallback);
    }
}

//--------------------------------------------------------------------------
void IBus::setMessageCallback(MessageCallback f)
{
    mMsgCallback = f;
}


#endif
