/* 
 * File:   buttons.h
 * Author: tevs
 *
 * Take care of OpenBM_E39's buttons:
 * Button states:
 *    - DOWN : button is down for more than two cycles
 *    - PRESSED   : button was just pressed, so previous tick it was up and now down
 *    - RELEASED  : button was just release, so previous tick it was down and now up
 * 
 * Created on June 1, 2010, 11:04 PM
 */

#ifndef _BUTTONS_H
#define	_BUTTONS_H

#include <base.h>

/*** Port expander adresses for buttons ***/
#define PORT_EXPANDER_ENCODER_ADD 0b01000000
#define PORT_EXPANDER_RADIO_ADD   0b01000010
#define PORT_EXPANDER_BMBT_ADD    0b01000100


#ifdef	__cplusplus
extern "C"
{
#endif

/*** Button definitions ***/
// (don't change this, specific to hardware !!!)
typedef uint8_t buttonIndex_t;

//right part
#define BUTTON_6         0
#define BUTTON_DISP      1
#define BUTTON_4         2
#define BUTTON_AM        3
#define BUTTON_5         4
#define BUTTON_INFO_R    5
#define BUTTON_RIGHT_STATE_MASK_1 0x3FL
#define BUTTON_3         6
#define BUTTON_MODE      7
#define BUTTON_1         8
#define BUTTON_FM        9
#define BUTTON_2         10
#define BUTTON_INFO_L    11
#define BUTTON_RIGHT_STATE_MASK_2 0xFC0L

// left part
#define BUTTON_TEL       12
#define BUTTON_SELECT    13
#define BUTTON_FF        14
#define BUTTON_UHR       15
#define BUTTON_MENU_LR   16
#define BUTTON_REW       17
#define BUTTON_TONE      18
#define BUTTON_PRG       19
#define BUTTON_LEFT_STATE_MASK 0xFF000L

// BMBT-Encoder
#define BUTTON_BMBT_KNOB            20
#define BUTTON_BMBT_ENCODER_OUT1    21
#define BUTTON_BMBT_ENCODER_OUT2    22
#define BUTTON_BMBT_CW              23
#define BUTTON_BMBT_CCW             24

// Radio-Encoder
#define BUTTON_RADIO_ENCODER_OUT1   25
#define BUTTON_RADIO_ENCODER_OUT2   26
#define BUTTON_RADIO_KNOB           27
#define BUTTON_RADIO_CW             28
#define BUTTON_RADIO_CCW            29

#define BUTTON_KNOBS_STATE_MASK      0x3FF00000L

// Eject button
#define BUTTON_EJECT     30

// device specific, ignore this
#define BUTTON_ACTIVE_RIGHT_PART    31


/**** Button State access  ***/
//typedef uint8_t   buttonState_t;
typedef uint32_t  buttonGlobalState_t;
typedef uint8_t   buttonBool_t;


/**
 * Init buttons
 **/
extern void button_init(void);

/**
 * Update state of buttons.
 **/
extern void button_tick(void);

/**
 * Return state of a one button. The state is encoded, so use
 * defined macros to extract button state
 **/
extern buttonBool_t button(buttonIndex_t id);
extern buttonBool_t button_down(buttonIndex_t id);
extern buttonBool_t button_pressed(buttonIndex_t id);
extern buttonBool_t button_released(buttonIndex_t id);


/**
 * Get according state of all buttons
 **/
extern buttonGlobalState_t button_global(void);
extern buttonGlobalState_t button_global_down(void);
extern buttonGlobalState_t button_global_pressed(void);
extern buttonGlobalState_t button_global_released(void);

#ifdef	__cplusplus
}
#endif

#endif	/* _BUTTONS_H */

