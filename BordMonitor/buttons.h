/* 
 * File:   buttons.h
 * Author: tevs
 *
 * Take care of OpenBM_E39's buttons:
 * Button states:
 *    - DOWN : button is down for more than two cycles
 *    - PRESSED   : button was just pressed, so previous tick it was up and now down
 *    - RELEASED  : button was just release, so previous tick it was down and now up
 *    - LONG_DOWN : button is down longer than specified time (usually 1s)
 *
 * Created on June 1, 2010, 11:04 PM
 */

#ifndef _BUTTONS_H
#define	_BUTTONS_H

#include <base.h>

/*** Port expander adresses for buttons ***/
#define PORT_EXPANDER_ENC_RADIO  0b10010010
#define PORT_EXPANDER_ENC_BMBT   0b01000000
#define PORT_EXPANDER_RADIO      0b01000100
#define PORT_EXPANDER_BMBT       0b01000010


#ifdef	__cplusplus
extern "C"
{
#endif

// delay in ticks, when we decide that button is hold too long
#define BUTTON_DELAY_LONG TICKS_PER_ONE_AND_A_HALF_SECONDS
    
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
#define BUTTON_SELECT    12
#define BUTTON_TEL       13
#define BUTTON_FF        14
#define BUTTON_UHR       15
#define BUTTON_MENU_LR   16
#define BUTTON_PRG       17
#define BUTTON_TONE      18
#define BUTTON_REW       19
#define BUTTON_LEFT_STATE_MASK 0xFF000L

// Eject button
#define BUTTON_EJECT     20

// Encoder knobs
#define BUTTON_BMBT_KNOB            21
#define BUTTON_RADIO_KNOB           22

// Number of all buttons, which's press time can be evaluated
#define BUTTON_NUM_BUTTONS 23

// Encoder IDs BMBT or RADIO
#define ENC_BMBT 0
#define ENC_RADIO 1

/**** Button State access  ***/
//typedef uint8_t   buttonState_t;
typedef uint32_t  buttonGlobalState_t;
typedef uint8_t   buttonBool_t;


/**
 * Init buttons
 **/
extern void button_init(void);

/**
 * Let button handler know, that it should check for buttons on the next tick
 **/
extern void button_isr(void);

/**
 * Update state of buttons. This method can be called as often as you want.
 * The frequency determines how well it will react on button changes.
 * NOTE: call this function on every main timer event, so calling this every 1/80sec
 * or so is enough
 **/
extern void button_tick(void);
extern void button_after_tick(void);

/**
 * This method should be called as often as you can. This will check for
 * encoder states. They are independent of other buttons and hence need some
 * critical timings.
 * NOTE: call this in your main loop (the execution time of this function is
 * approximately short, because it do long execution only if rotary state changed)
 **/
extern void button_tick_encoder(void);

/**
 * Return current encoder states as integer. negative means, the movement is counterclockwise.
 * The encID is either ENC_RADIO or ENC_BMBT
 **/
extern int8_t button_encoder(uint8_t encID);

/**
 * Return state of a one button. The state is encoded, so use
 * defined macros to extract button state.
 * Long-
 **/
extern buttonBool_t button(buttonIndex_t id);
extern buttonBool_t button_down(buttonIndex_t id);
extern buttonBool_t button_pressed(buttonIndex_t id);
extern buttonBool_t button_released(buttonIndex_t id);

// return first detected button which is down (return NUM_BUTTONS if nothing is down)
extern buttonIndex_t button_whichDown(void);

// return time in ticks how long a button is already down
extern uint8_t button_down_time(buttonIndex_t id);

// return if button is down for longer than specified long delay
#define button_down_long(id) (button_down_time(id) >= BUTTON_DELAY_LONG)


#ifdef	__cplusplus
}
#endif

#endif	/* _BUTTONS_H */

