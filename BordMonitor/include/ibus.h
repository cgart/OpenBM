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

#ifdef __cplusplus
extern "C" {
#endif

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
#define IBUS_MSG_UPDATE_MID_BOTTOM  0x21    // update information on text display
#define IBUS_MSG_UPDATE_MID_TOP     0x23    // update information on text display
#define IBUS_MSG_RADIO_ENCODER      0x32    // MID's radio encoder was rotated
#define IBUS_MSG_BMBT_ENCODER       0x3B    // MID's radio encoder was rotated
#define IBUS_MSG_BUTTON             0x31    // MID's button state change


//*** iBus Settings ***
#define IBUS_TX_SETUP()               { DDRD |= (1 << DDD1); PORTD |= (1 << 1); }
#define IBUS_TX_PORT                  PORTD
#define IBUS_TX_PIN                   1
#define IBUS_BAUD_DELAY()             { _delay_us(52); _delay_us(52); }

#define IBUS_SENSTA_VALUE()           bit_is_set(PIND,2)
#define IBUS_SENSTA_SETUP()           { DDRD &= ~(1 << DDD2); PORTD |= (1 << 2); }

// setup timer used for ibus timings
#define IBUS_TIMER_SETUP() { TCCR1B = (1 << CS11) | (1 << CS10); }

// wait time by collision when transmitting (around 5.0ms)
#define IBUS_TIMEOUT_COLLISION() { BEGIN_ATOMAR; TCNT1 = 65535 - 1650; TIMSK |= (1 << TOIE1); END_ATOMAR; }

// receive timeout (stop receiving when nothing happens) (around 50ms)
#define IBUS_TIMEOUT_RECEIVE() { BEGIN_ATOMAR; TCNT1 = 65535 - 11500; TIMSK |= (1 << TOIE1); END_ATOMAR; }

// wait time when receive error (around 2.0ms)
#define IBUS_TIMEOUT_RECEIVE_ERROR() { BEGIN_ATOMAR; TCNT1 = 65535 - 460; TIMSK |= (1 << TOIE1); END_ATOMAR; }

// wait when message was transmitted before next message will be transmitted around 2ms
#define IBUS_TIMEOUT_AFTER_TRANSMIT() { BEGIN_ATOMAR; TCNT1 = 65535 - 460; TIMSK |= (1 << TOIE1); END_ATOMAR; }

// if we see busy bus, then we wait at least 2.0 ms
#define IBUS_TIMEOUT_WAIT_FREE_BUS() { BEGIN_ATOMAR; TCNT1 = 65535 - 460; TIMSK |= (1 << TOIE1); END_ATOMAR; }

#define IBUS_TIMER_DISABLE_INTERRUPT() { TIMSK &= ~(1 << TOIE1); }
#define IBUS_TIMER_INTERRUPT TIMER1_OVF_vect
#define IBUS_TRANSMIT_TRIES 5

//******* default constants **********
#define IBUS_STATE_IDLE 0
#define IBUS_STATE_WAIT_FREE_BUS  (1 << 0)
#define IBUS_STATE_RECEIVING  (1 << 1)

/**
 * Class to handle IBus messages. The I-Bus must be conencted
 * through a proper device to the uart connecter of the µC.
 * This device must handle the bus specific communication.
 * The logic is handled in this class blah blah blah :-)
 **/

/**
 * Init ibus interface. This will also initialise the used uart interface.
 */
extern void ibus_init(void);

/**
 * Set callback function, called when message arived.
 * the ibus is found.
 * @param src ID of the sender
 * @param dst ID of the receiver
 * @param msg String containing the full message
 * @param msglen Length of the message string
 **/
extern void ibus_setMessageCallback(void(*cb)(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen));

/**
 * Update ibus instance.
 **/
extern void ibus_tick(void);

/**
 * Send message over the ibus. You can speify how much tries
 * there will be if message could not be send.
 **/
extern void ibus_sendMessage(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msgLength, uint8_t numberOfTries);


 #ifdef __cplusplus
 }
 #endif


#endif
