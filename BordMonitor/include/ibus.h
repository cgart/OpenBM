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
#define IBUS_DEV_RLS     0xE8    // Rain/Light-Sensor
#define IBUS_DEV_TV      0xED    // Television
#define IBUS_DEV_BMBT    0xF0    // On-board monitor operating part
#define IBUS_DEV_CSU     0xF5    // unknown
#define IBUS_DEV_LOC     0xFF    // Local


//*** Message Types ***
#define IBUS_MSG_DEV_POLL           0x01    // Poll any device
#define IBUS_MSG_DEV_READY          0x02    // answer to poll message
#define IBUS_MSG_VEHICLE_CTRL_REQ   0x0B    // Request state of the diag data
#define IBUS_MSG_VEHICLE_CTRL       0x0C    // Vehicle Control (mostly used from diagnose)
#define IBUS_MSG_IGNITION_REQ       0x10    // Request ignition state
#define IBUS_MSG_IGNITION           0x11    // State of the ignition
#define IBUS_MSG_IKE_STATE_REQ      0x12    // Request current IKE state
#define IBUS_MSG_IKE_STATE          0x13    // IKE informs about its state
#define IBUS_MSG_IKE_SPEED          0x18    // IKE informs about the speed and RPM
#define IBUS_MSG_MID_STATE_BUTTONS  0x20    // MID's main buttons (on/off, TEL, ...)
#define IBUS_MSG_UPDATE_MID_BOTTOM  0x21    // update information on text display
#define IBUS_MSG_MID_ACK_TEXT       0x22    // mid aknowledges that text was received
#define IBUS_MSG_UPDATE_MID_TOP     0x23    // update information on text display
#define IBUS_MSG_UPDATE_MID_TOP_FREQ 0x24   // update frequency field of the radio
#define IBUS_MSG_LED                0x2B    // set status-LED state
#define IBUS_MSG_LED_SPECIAL        0x2D    // set status-LED state (special function, defining blink ratio)
#define IBUS_MSG_BUTTON             0x31    // MID's button state change
#define IBUS_MSG_RADIO_ENCODER      0x32    // MID's radio encoder was rotated
#define IBUS_MSG_MFL_BUTTON         0x3B    // MFL's button state change
#define IBUS_MSG_BMBT_BUTTON        0x48    // action with BMBT button
#define IBUS_MSG_BMBT_ENCODER       0x49    // BMBT encoder was rotated
#define IBUS_MSG_LAMP_STATE         0x5B    // Lamp state
#define IBUS_MSG_DIMMER_STATE       0x5C    // Dimmer state
#define IBUS_MSG_GM_KEY_BUTTON      0x72    // state of the buttons on the key
#define IBUS_MSG_GM_ENABLE_STATE    0x76    // state of GM indicating if car is closed or not ??? could be DWA?!
#define IBUS_MSG_GM_STATE_REQ       0x79    // request current state of the GM (doors, trunk, ...)
#define IBUS_MSG_GM_STATE           0x7A    // state of the GM (doors, trunk, ...)
#define IBUS_MSG_DIA_ACK            0xA0    // acknowledge diagnose message


#define IBUS_MSG_OPENBM_TO          0xFA    // message sent to OpenBM (not BMW specified)
#define IBUS_MSG_OPENBM_FROM        0xFB    // message received from OpenBM (not BMW specified)

#define IBUS_MSG_OPENBM_GET_VERSION 0x00    // second data byte: get version of the firmware
#define IBUS_MSG_OPENBM_GET_TICKS   0x01    // second data byte: get number of ticks
#define IBUS_MSG_OPENBM_GET_PHOTO   0x02    // second data byte: get value of the photo sensor
#define IBUS_MSG_OPENBM_GET_DIMMER  0x03    // second data byte: get value of the backlight dimmer
#define IBUS_MSG_OPENBM_GET_TEMP    0x04    // second data byte: get value of the temperature sensor
#define IBUS_MSG_OPENBM_SET_DISPLAY 0x10    // set display input (next byte corresponds directly to the input, half upper byte = power (0000 - nothing, 1111 - on, 0110 - off), half lower byte = input)
#define IBUS_MSG_OPENBM_SET_DISPLAY_LIGHT 0x11 // set brightness of display (warning automatic brightness must be disabled)
#define IBUS_MSG_OPENBM_OBMS_SET    0x12    // setup settings for OBMS
#define IBUS_MSG_OPENBM_SET_PHOTO   0x13    // setup settings for the photo sensor
#define IBUS_MSG_OPENBM_SPECIAL_REQ 0xFF    // second data byte: special request message (i.e. update firmware)
#define IBUS_MSG_OPENBM_SETTINGS    0xFE    // second data byte: write/read settings of OpenBM to/from EEPROM

//*** iBus Settings ***
#define IBUS_TX_SETUP()               { DDRD |= (1 << DDD1); PORTD |= (1 << 1); }
#define IBUS_TX_PORT                  PORTD
#define IBUS_TX_PIN                   1
#define IBUS_BAUD_DELAY()             { _delay_us(52); _delay_us(52); }

#define IBUS_SENSTA_VALUE()           bit_is_set(PIND,2)
#define IBUS_SENSTA_SETUP()           { DDRD &= ~(1 << DDD2); PORTD |= (1 << 2); }

#define IBUS_TIMER0

#ifdef IBUS_TIMER0
    #define IBUS_TIMER_SETUP() { TCCR0B = (1 << CS02) | (1 << CS00); }

    // wait time by collision when transmitting (around 5.0ms)
    #define IBUS_TIMEOUT_COLLISION() ibus_setTimeOut(80);

    // receive timeout (stop receiving when nothing happens) (around 17ms)
    #define IBUS_TIMEOUT_RECEIVE() ibus_setTimeOut(255);

    // wait time when receive error (around 2.7ms)
    #define IBUS_TIMEOUT_RECEIVE_ERROR() ibus_setTimeOut(40);

    // wait when message was transmitted before next message will be transmitted around 2ms
    #define IBUS_TIMEOUT_AFTER_TRANSMIT() ibus_setTimeOut(30);

    // if we just have received something, then wait until next ibus operation 2.0 ms
    #define IBUS_TIMEOUT_AFTER_RECEIVE() ibus_setTimeOut(30);

    // if we see busy bus, then we wait at least 3.0 ms
    #define IBUS_TIMEOUT_WAIT_FREE_BUS() ibus_setTimeOut(50);

    #define IBUS_TIMER_DISABLE_INTERRUPT() { TIMSK0 &= ~(1 << TOIE0); TIFR0 |= (1 << TOV0); }
    #define IBUS_TIMER_INTERRUPT TIMER0_OVF_vect

#else

    // setup timer used for ibus timings
    #define IBUS_TIMER_SETUP() { TCCR1B = (1 << CS11) | (1 << CS10); }

    // wait time by collision when transmitting (around 5.0ms)
    #define IBUS_TIMEOUT_COLLISION() { TCNT1 = 65535 - 1650; TIMSK1 |= (1 << TOIE1); TIFR1 |= (1 << TOV1); }

    // receive timeout (stop receiving when nothing happens) (around 50ms)
    #define IBUS_TIMEOUT_RECEIVE() { TCNT1 = 65535 - 11500; TIMSK1 |= (1 << TOIE1); TIFR1 |= (1 << TOV1);}

    // wait time when receive error (around 2.0ms)
    #define IBUS_TIMEOUT_RECEIVE_ERROR() { TCNT1 = 65535 - 460; TIMSK1 |= (1 << TOIE1); TIFR1 |= (1 << TOV1);}

    // wait when message was transmitted before next message will be transmitted around 2ms
    #define IBUS_TIMEOUT_AFTER_TRANSMIT() { TCNT1 = 65535 - 460; TIMSK1 |= (1 << TOIE1); TIFR1 |= (1 << TOV1);}

    // if we see busy bus, then we wait at least 2.0 ms
    #define IBUS_TIMEOUT_WAIT_FREE_BUS() { TCNT1 = 65535 - 460; TIMSK1 |= (1 << TOIE1);  TIFR1 |= (1 << TOV1);}

    #define IBUS_TIMER_DISABLE_INTERRUPT() { TIMSK1 &= ~(1 << TOIE1); TIFR1 |= (1 << TOV1); }
    #define IBUS_TIMER_INTERRUPT TIMER1_OVF_vect
#endif
    

//******* default constants **********
#define IBUS_TRANSMIT_TRIES 5
#define IBUS_STATE_IDLE 0
#define IBUS_STATE_WAIT_FREE_BUS  (1 << 0)
#define IBUS_STATE_RECEIVING  (1 << 1)
#define IBUS_STATE_TRANSMITTING  (1 << 2)

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

/**
 * Check whenever ibus transmit queue is empty.
 **/
extern uint8_t ibus_isQueueFree(void);

/**
 * Return 1 if on the next ibus_tick() message will be transmitted
 **/
extern uint8_t ibus_readyToTransmit(void);

 #ifdef __cplusplus
 }
 #endif


#endif
