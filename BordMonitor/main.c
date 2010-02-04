/* 
 * File:   main.c
 * Author: tevs
 *
 * Created on December 10, 2009, 12:24 AM
 */

#include "base.h"
#include <i2cmaster.h>

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
    char bMENU_LR;
    //char bMENU_R;
    char bTONE;
    char bPRG;
    //char bEJECT;

    // BMBT-Encoder
    char bmbt_bButton;
    char bmbt_bCW;
    char bmbt_bCCW;

    // Radio-Encoder
    char radio_bButton;
    char radio_bRotate;
    char radio_bCW;

}ButtonState;

volatile ButtonState g_ButtonState;
ButtonState g_oldState;

#define SCL_CLOCK 20000 // 20kHz for the I2C Bus
#define PORT_EXPANDER_ADD 0x40


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

//volatile char gRadioEncoder_PinA = 0;

volatile char state = 0;

//------------------------------------------------------------------------------
// Interrupt routine to react on PCF8574 interrupts
// PCF8574 is acting inbetween both encoders of the board and mc
//------------------------------------------------------------------------------
ISR(INT0_vect)
{
    if (state) state = 0;
    else state = 1;

    BG_LED(state);

    // clear bit now, so that it get not reset when get back from atomar mode
    TWCR = (1 << TWINT);

    BEGIN_ATOMAR;
    {
        i2c_start_wait(0x40 + I2C_READ);
        register unsigned char portState = i2c_readNak();
        i2c_stop();

        // bmbt encoder
        if (portState & (1 << 3)) g_ButtonState.bmbt_bButton = 0;
        else g_ButtonState.bmbt_bButton = 1;


        // radio encoder
        if (portState & (1 << 4)) g_ButtonState.radio_bButton = 0;
        else g_ButtonState.radio_bButton = 1;

        /*if (portState & (1 << 5)) g_ButtonState.radio_bRotate = 0;
        else g_ButtonState.radio_bRotate = 1;

        if (portState & (1 << 6))
        {
            if (g_ButtonState.radio_bRotate)
                g_ButtonState.radio_bCW = 1;
            else
                g_ButtonState.radio_bCW = 0;
        }*/

        // bmbt P2, P1 (rotary), P3 (button)
        // radio P5, P6 (rotary), P4 (button)
    }
    END_ATOMAR;

}


//------------------------------------------------------------------------------
// Init all hardware parts needed for the project
//------------------------------------------------------------------------------
void initHardware(void)
{
    // init I2C, port expander
    TWSR = 0;
    TWBR = ((F_CPU/SCL_CLOCK)-16)/2;
    MCUCR |= (1 << ISC01); // react on falling edge
    MCUCR &= ~(1 << ISC00);
    GICR |= (1 << INT0);

    
    // enable interrupts
    sei();
}

// state of twi actions
typedef enum _TWIstate
{
    IDLE = 0,
    START,
    READ,
    STOP
}TWIstate;
volatile TWIstate g_TWIstate;

//------------------------------------------------------------------------------
// React on interrupts while I2C transfer
//------------------------------------------------------------------------------
ISR(TWI_vect)
{

}

//------------------------------------------------------------------------------
// Check current state of the PCF8574 port expander responsible for the both
// encoders (return state of the 8 pins, 0 if error)
// Returned 0 means an error, so make sure that at least one PIN of the port
// expander pulled-up to 1 to make sure that 0 do never happens
//------------------------------------------------------------------------------
uint8_t getEncoderState(void)
{
    if (g_TWIstate != IDLE) return 0;

    // first send start condition
    switch (g_TWIstate)
    {
    case IDLE:
        g_TWIstate = START;
        TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);        
        break;
    case START:
        break;
    case READ:
        break;
    case STOP:
        break;
    }
    
}

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
    DDRA &= ~(1 << 6); // input on portA.5
    PORTA |= (1 << 6); // pull-up on portA.5

    DDRD &= ~(1 << 2);
    PORTD |= (1 << 2);
    //DDRC &= ~(1 << 0) & ~(1 << 1);
    PORTC &= ~(1 << 0) & ~(1 << 1); // disable pull-ups on prortC.0/1

    // display button output
    DDRA |= (1 << DDA7);
    PORTA |= (1 << 7);
    
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
    g_ButtonState.bMENU_LR = 0;
    //g_ButtonState.bMENU_R = 0;
    g_ButtonState.bTONE = 0;
    g_ButtonState.bPRG = 0;
    //g_ButtonState.bEJECT = 0;

    _delay_ms(100);

    //const unsigned char FIRST_ON = ((~(1 << 6))) | (1 << 7);
    //const unsigned char SECOND_ON = ((~(1 << 7))) | (1 << 6);

    // init PCF8574
    i2c_init();
    //i2c_start_wait(0x40 + I2C_WRITE);
    //i2c_write(0xFF);
    //i2c_stop();
    
    sei();
    GICR |= (1<<INT0);

    for(;;)
    {

        g_oldState = g_ButtonState;
        
        // get current state of all the buttons on the right
        {
            //PORTC = FIRST_ON;
            PORTC &= ~(1 << 6);
            PORTC |= (1 << 7);
            _delay_ms(1);
            
            g_ButtonState.bDISP = bit_is_clear(PINB, 0);
            g_ButtonState.b6 = bit_is_clear(PINB, 1);
            g_ButtonState.b4 =  bit_is_clear(PINC, 2);
            g_ButtonState.bAM =  bit_is_clear(PINC, 3);
            g_ButtonState.b5 =  bit_is_clear(PINC, 4);
            g_ButtonState.bINFO_R =  bit_is_clear(PINC, 5);
            
            //PORTC = SECOND_ON;
            PORTC &= ~(1 << 7);
            PORTC |= (1 << 6);
            _delay_ms(1);
            
            g_ButtonState.bMODE = bit_is_clear(PINB, 0);
            g_ButtonState.b3 = bit_is_clear(PINB, 1);
            g_ButtonState.b1 = bit_is_clear(PINC, 2);
            g_ButtonState.bFM = bit_is_clear(PINC, 3);
            g_ButtonState.b2 = bit_is_clear(PINC, 4);
            g_ButtonState.bINFO_L = bit_is_clear(PINC, 5);
        }

        // now check the buttons on the right side
        {
            //g_ButtonState.bEJECT = bit_is_clear(PINB, 0);
            //g_ButtonState.bMENU_L = bit_is_clear(PINB, 1);
            g_ButtonState.bSELECT = bit_is_clear(PINB, 2);
            g_ButtonState.bFF = bit_is_clear(PINB, 3);
            g_ButtonState.bUHR = bit_is_clear(PINB, 4);
            g_ButtonState.bTEL = bit_is_clear(PINB, 5);
            g_ButtonState.bPRG = bit_is_clear(PINB, 6);
            g_ButtonState.bTONE = bit_is_clear(PINB, 7);
            g_ButtonState.bREW = bit_is_clear(PINA, 6);
            g_ButtonState.bMENU_LR = bit_is_clear(PIND, 4);
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

        // push display button (toggle between video inputs)
        if (g_ButtonState.bDISP)
            PORTA &= ~(1 << 7);
        else
            PORTA |= (1 << 7);


        if (g_oldState.bmbt_bButton != g_ButtonState.bmbt_bButton)
        {
            R_LED(g_ButtonState.bmbt_bButton)
        }

        if (g_oldState.radio_bButton != g_ButtonState.radio_bButton)
        {
            G_LED(g_ButtonState.radio_bButton)
        }


        /*if (g_oldState.radio_bRotate != g_ButtonState.radio_bRotate)
        {
            if (g_ButtonState.radio_bCW)
            {
                R_LED_ON();
                Y_LED_OFF();
            }else
            {
                R_LED_OFF();
                Y_LED_ON();
            }
        }*/
    }

    cli();
    
    return 0;
}

