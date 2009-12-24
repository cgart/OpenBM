/* 
 * File:   main.c
 * Author: tevs
 *
 * Created on December 10, 2009, 12:24 AM
 */

#include "base.h"

typedef struct _ButtonState
{
    //right part
    char bDISP;
    char bMODE;
    char bAM;
    char bFM;
    char b6;
    char b3;
    char b5;
    char b2;
    char b4;
    char b1;
    char bINFO_R;
    char bINFO_L;

    // left part
    char bUHR;
    char bTEL;
    char bFF;
    char bREW;
    char bSELECT;
    char bMENU_L;
    char bMENU_R;
    char bTONE;
    char bPRG;
    char bEJECT;
}ButtonState;

ButtonState g_ButtonState;
ButtonState g_oldState;

#define SETUP_BG_LED(a) {DDRA |= (1 << DDA0); PORTA &= ~(1 << 0); }
#define SETUP_R_LED(a)  {DDRA |= (1 << DDA1); PORTA &= ~(1 << 1); }
#define SETUP_G_LED(a)  {DDRA |= (1 << DDA2); PORTA &= ~(1 << 2); }
#define SETUP_Y_LED(a) {DDRD |= (1 << DDA5); PORTD &= ~(1 << 5); }
#define SETUP_FAN_LED(a) {DDRD |= (1 << DDD6); PORTD &= ~(1 << 6); }
#define SETUP_RADIO_LED(a) {DDRD |= (1 << DDD7); PORTD &= ~(1 << 7); }

#define BG_LED_ON() {PORTA |= (1 << 0);}
#define BG_LED_OFF() {PORTA &= ~(1 << 0);}
#define BG_LED(a) {if(a){ BG_LED_ON(); }else{ BG_LED_OFF();}}

#define R_LED_ON() {PORTA |= (1 << 1);}
#define R_LED_OFF() {PORTA &= ~(1 << 1);}
#define R_LED(a) { if(a){ R_LED_ON(); } else{ R_LED_OFF();} }

#define G_LED_ON() {PORTA |= (1 << 2);}
#define G_LED_OFF() {PORTA &= ~(1 << 2);}
#define G_LED(a) {if(a){ G_LED_ON(); }else{ G_LED_OFF();}}

#define Y_LED_ON() {PORTD |= (1 << 5);}
#define Y_LED_OFF() {PORTD &= ~(1 << 5);}
#define Y_LED(a) {if(a){ Y_LED_ON();} else{ Y_LED_OFF();}}

#define FAN_LED_ON() {PORTD |= (1 << 6);}
#define FAN_LED_OFF() {PORTD &= ~(1 << 6);}
#define FAN_LED(a) {if(a){ FAN_LED_ON();} else{ FAN_LED_OFF();}}

#define RADIO_LED_ON() {PORTD |= (1 << 7);}
#define RADIO_LED_OFF() {PORTD &= ~(1 << 7);}
#define RADIO_LED(a) {if(a){ RADIO_LED_ON(); }else{ RADIO_LED_OFF();}}

//------------------------------------------------------------------------------
// Set FUSES correctrly ( http://www.engbedded.com/fusecalc/):
// avrdude -U lfuse:w:0xcf:m -U hfuse:w:0xd9:m  (ext crystal, disable jtag)
//------------------------------------------------------------------------------
int main(void)
{
    // setup default values
    DDRA = 0;
    DDRB = 0;
    DDRC = 0;
    DDRD = 0;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    
    // setup LEDs
    SETUP_BG_LED();
    SETUP_R_LED();
    SETUP_G_LED();
    SETUP_Y_LED();
    SETUP_FAN_LED();
    SETUP_RADIO_LED();
    
    // This are the check lines for the one part of the buttons
    DDRC = (1 << DDC6) | (1 << DDC7);  // full C port as input, only pin 6 and 7 as outputs
    PORTC = 0xFF;   // enable pull-up on all inputs
    DDRB = 0; // port B as input
    PORTB = 0xFF; // pull-ups on port B
    DDRD &= ~(1 << 3) & ~(1 << 4);
    PORTD |= (1 << 3) | (1 << 4);

    // init default values
    g_ButtonState.b1 = 0;
    g_ButtonState.b2 = 0;
    g_ButtonState.b3 = 0;
    g_ButtonState.b4 = 0;
    g_ButtonState.b5 = 0;
    g_ButtonState.b6 = 0;
    g_ButtonState.bAM = 0;
    g_ButtonState.bDISP = 0;
    g_ButtonState.bFM = 0;
    g_ButtonState.bINFO_L = 0;
    g_ButtonState.bINFO_R = 0;
    g_ButtonState.bMODE = 0;
    g_ButtonState.bUHR = 0;
    g_ButtonState.bTEL = 0;
    g_ButtonState.bFF = 0;
    g_ButtonState.bREW = 0;
    g_ButtonState.bSELECT = 0;
    g_ButtonState.bMENU_L = 0;
    g_ButtonState.bMENU_R = 0;
    g_ButtonState.bTONE = 0;
    g_ButtonState.bPRG = 0;
    g_ButtonState.bEJECT = 0;

    _delay_ms(100);

    const unsigned char FIRST_ON = (0xFF & (~(1 << 6))) | (1 << 7);
    const unsigned char SECOND_ON = (0xFF & (~(1 << 7))) | (1 << 6);
    
    for(;;)
    {

        g_oldState = g_ButtonState;
        
        // get current state of all the buttons on the right
        {
            PORTC = FIRST_ON;
            _delay_ms(1);
            
            g_ButtonState.bDISP = bit_is_clear(PINC, 0);
            g_ButtonState.b6 = bit_is_clear(PINC, 1);
            g_ButtonState.b4 =  bit_is_clear(PINC, 2);
            g_ButtonState.bAM =  bit_is_clear(PINC, 3);
            g_ButtonState.b5 =  bit_is_clear(PINC, 4);
            g_ButtonState.bINFO_R =  bit_is_clear(PINC, 5);
            
            PORTC = SECOND_ON;
            _delay_ms(1);
            
            g_ButtonState.bMODE = bit_is_clear(PINC, 0);
            g_ButtonState.b3 = bit_is_clear(PINC, 1);
            g_ButtonState.b1 = bit_is_clear(PINC, 2);
            g_ButtonState.bFM = bit_is_clear(PINC, 3);
            g_ButtonState.b2 = bit_is_clear(PINC, 4);
            g_ButtonState.bINFO_L = bit_is_clear(PINC, 5);
        }

        // now check the buttons on the right side
        {
            g_ButtonState.bEJECT = bit_is_clear(PINB, 0);
            g_ButtonState.bMENU_L = bit_is_clear(PINB, 1);
            g_ButtonState.bSELECT = bit_is_clear(PINB, 2);
            g_ButtonState.bFF = bit_is_clear(PINB, 3);
            g_ButtonState.bUHR = bit_is_clear(PINB, 4);
            g_ButtonState.bTEL = bit_is_clear(PINB, 5);
            g_ButtonState.bPRG = bit_is_clear(PINB, 6);
            g_ButtonState.bTONE = bit_is_clear(PINB, 7);
            g_ButtonState.bREW = bit_is_clear(PIND, 3);
            g_ButtonState.bMENU_R = bit_is_clear(PIND, 4);
        }

        // react on the state change accordingly
        if (g_oldState.bUHR != g_ButtonState.bUHR)
        {
            R_LED(g_ButtonState.bUHR);
        }
        if (g_oldState.bTEL != g_ButtonState.bTEL)
        {
            G_LED(g_ButtonState.bTEL);
        }
        if (g_oldState.bPRG != g_ButtonState.bPRG)
        {
            Y_LED(g_ButtonState.bPRG);
        }
        if (g_oldState.bTONE != g_ButtonState.bTONE)
        {
            FAN_LED(g_ButtonState.bTONE);
        }
        if (g_oldState.bFF != g_ButtonState.bFF)
        {
            RADIO_LED(g_ButtonState.bFF);
        }
        if (g_oldState.bREW != g_ButtonState.bREW)
        {
            BG_LED(g_ButtonState.bREW);
        }

    }

    return 0;
}

